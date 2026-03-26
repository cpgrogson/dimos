# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from abc import ABC
from collections import defaultdict
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from functools import cached_property, reduce
import operator
import sys
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Literal, get_args, get_origin, get_type_hints

if TYPE_CHECKING:
    from dimos.protocol.service.system_configurator.base import SystemConfigurator

from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module, ModuleBase, ModuleSpec, is_module_type
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, PubSubTransport, pLCMTransport
from dimos.spec.utils import Spec, is_spec, spec_annotation_compliance, spec_structural_compliance
from dimos.utils.generic import short_id
from dimos.utils.logging_config import setup_logger

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

logger = setup_logger()


@dataclass(frozen=True)
class StreamRef:
    name: str
    type: type
    direction: Literal["in", "out"]


@dataclass(frozen=True)
class ModuleRef:
    name: str
    spec: type[Spec] | type[ModuleBase]


@dataclass(frozen=True)
class StreamWiring:
    """Compiled instruction: set a transport on a module's stream."""

    module_class: type[ModuleBase]
    stream_name: str
    transport: PubSubTransport[Any]


@dataclass(frozen=True)
class ModuleRefWiring:
    """Compiled instruction: link base_module.ref_name → target_module."""

    base_module: type[ModuleBase]
    ref_name: str
    target_module: type[ModuleBase]


@dataclass(frozen=True)
class RpcWiringPlan:
    """Compiled RPC wiring: registry of methods + per-module binding requests."""

    # rpc_key -> (module_class, method_name) — the full callable registry
    registry: dict[str, tuple[type[ModuleBase], str]]
    # (module_class, set_method_name, linked_rpc_key) — for set_X pattern
    set_methods: tuple[tuple[type[ModuleBase], str, str], ...]
    # (module_class, requested_name, rpc_key) — for rpc_calls pattern
    rpc_call_bindings: tuple[tuple[type[ModuleBase], str, str], ...]


@dataclass(frozen=True)
class _BlueprintAtom:
    kwargs: dict[str, Any]
    module: type[ModuleBase[Any]]
    streams: tuple[StreamRef, ...]
    module_refs: tuple[ModuleRef, ...]

    @classmethod
    def create(cls, module: type[ModuleBase[Any]], kwargs: dict[str, Any]) -> Self:
        streams: list[StreamRef] = []
        module_refs: list[ModuleRef] = []

        # Resolve annotations using namespaces from the full MRO chain so that
        # In/Out behind TYPE_CHECKING + `from __future__ import annotations` work.
        # Iterate reversed MRO so the most specific class's namespace wins when
        # parent modules shadow names (e.g. spec.perception.Image vs sensor_msgs.Image).
        globalns: dict[str, Any] = {}
        for c in reversed(module.__mro__):
            if c.__module__ in sys.modules:
                globalns.update(sys.modules[c.__module__].__dict__)
        try:
            all_annotations = get_type_hints(module, globalns=globalns)
        except Exception:
            # Fallback to raw annotations if get_type_hints fails.
            all_annotations = {}
            for base_class in reversed(module.__mro__):
                if hasattr(base_class, "__annotations__"):
                    all_annotations.update(base_class.__annotations__)

        for name, annotation in all_annotations.items():
            origin = get_origin(annotation)
            # Streams
            if origin in (In, Out):
                direction = "in" if origin == In else "out"
                type_ = get_args(annotation)[0]
                streams.append(
                    StreamRef(name=name, type=type_, direction=direction)  # type: ignore[arg-type]
                )
            # linking to unknown module via Spec
            elif is_spec(annotation):
                module_refs.append(ModuleRef(name=name, spec=annotation))
            # linking to specific/known module directly
            elif is_module_type(annotation):
                module_refs.append(ModuleRef(name=name, spec=annotation))

        return cls(
            module=module,
            streams=tuple(streams),
            module_refs=tuple(module_refs),
            kwargs=kwargs,
        )


@dataclass(frozen=True)
class Blueprint:
    blueprints: tuple[_BlueprintAtom, ...]
    disabled_modules_tuple: tuple[type[ModuleBase], ...] = field(default_factory=tuple)
    transport_map: Mapping[tuple[str, type], PubSubTransport[Any]] = field(
        default_factory=lambda: MappingProxyType({})
    )
    global_config_overrides: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))
    remapping_map: Mapping[tuple[type[ModuleBase], str], str | type[ModuleBase] | type[Spec]] = (
        field(default_factory=lambda: MappingProxyType({}))
    )
    requirement_checks: tuple[Callable[[], str | None], ...] = field(default_factory=tuple)
    configurator_checks: "tuple[SystemConfigurator, ...]" = field(default_factory=tuple)

    @classmethod
    def create(cls, module: type[ModuleBase], **kwargs: Any) -> "Blueprint":
        blueprint = _BlueprintAtom.create(module, kwargs)
        return cls(blueprints=(blueprint,))

    def disabled_modules(self, *modules: type[ModuleBase]) -> "Blueprint":
        return replace(self, disabled_modules_tuple=self.disabled_modules_tuple + modules)

    def transports(self, transports: dict[tuple[str, type], Any]) -> "Blueprint":
        return replace(self, transport_map=MappingProxyType({**self.transport_map, **transports}))

    def global_config(self, **kwargs: Any) -> "Blueprint":
        return replace(
            self,
            global_config_overrides=MappingProxyType({**self.global_config_overrides, **kwargs}),
        )

    def remappings(
        self,
        remappings: list[
            tuple[type[ModuleBase[Any]], str, str | type[ModuleBase[Any]] | type[Spec]]
        ],
    ) -> "Blueprint":
        remappings_dict = dict(self.remapping_map)
        for module, old, new in remappings:
            remappings_dict[(module, old)] = new
        return replace(self, remapping_map=MappingProxyType(remappings_dict))

    def requirements(self, *checks: Callable[[], str | None]) -> "Blueprint":
        return replace(self, requirement_checks=self.requirement_checks + tuple(checks))

    def configurators(self, *checks: "SystemConfigurator") -> "Blueprint":
        return replace(self, configurator_checks=self.configurator_checks + tuple(checks))

    @cached_property
    def _active_blueprints(self) -> tuple[_BlueprintAtom, ...]:
        if not self.disabled_modules_tuple:
            return self.blueprints
        disabled = set(self.disabled_modules_tuple)
        return tuple(bp for bp in self.blueprints if bp.module not in disabled)

    def _check_ambiguity(
        self,
        requested_method_name: str,
        interface_methods: Mapping[str, list[tuple[type[ModuleBase], str]]],
        requesting_module: type[ModuleBase],
    ) -> None:
        if (
            requested_method_name in interface_methods
            and len(interface_methods[requested_method_name]) > 1
        ):
            modules_str = ", ".join(
                impl[0].__name__ for impl in interface_methods[requested_method_name]
            )
            raise ValueError(
                f"Ambiguous RPC method '{requested_method_name}' requested by "
                f"{requesting_module.__name__}. Multiple implementations found: "
                f"{modules_str}. Please use a concrete class name instead."
            )

    def _get_transport_for(self, name: str, stream_type: type) -> PubSubTransport[Any]:
        transport = self.transport_map.get((name, stream_type), None)
        if transport:
            return transport

        use_pickled = getattr(stream_type, "lcm_encode", None) is None
        topic = f"/{name}" if self._is_name_unique(name) else f"/{short_id()}"
        return pLCMTransport(topic) if use_pickled else LCMTransport(topic, stream_type)

    @cached_property
    def _all_name_types(self) -> set[tuple[str, type]]:
        result = set()
        for blueprint in self._active_blueprints:
            for conn in blueprint.streams:
                remapped_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                if isinstance(remapped_name, str):
                    result.add((remapped_name, conn.type))
        return result

    def _is_name_unique(self, name: str) -> bool:
        return sum(1 for n, _ in self._all_name_types if n == name) == 1

    def _run_configurators(self) -> None:
        from dimos.protocol.service.system_configurator.base import configure_system
        from dimos.protocol.service.system_configurator.lcm_config import lcm_configurators

        configurators = [*lcm_configurators(), *self.configurator_checks]

        try:
            configure_system(configurators)
        except SystemExit:
            labels = [type(c).__name__ for c in configurators]
            print(
                f"Required system configuration was declined: {', '.join(labels)}",
                file=sys.stderr,
            )
            sys.exit(1)

    def _check_requirements(self) -> None:
        errors = []
        red = "\033[31m"
        reset = "\033[0m"

        for check in self.requirement_checks:
            error = check()
            if error:
                errors.append(error)

        if errors:
            for error in errors:
                print(f"{red}Error: {error}{reset}", file=sys.stderr)
            sys.exit(1)

    def _verify_no_name_conflicts(self) -> None:
        name_to_types = defaultdict(set)
        name_to_modules = defaultdict(list)

        for blueprint in self._active_blueprints:
            for conn in blueprint.streams:
                stream_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                name_to_types[stream_name].add(conn.type)
                name_to_modules[stream_name].append((blueprint.module, conn.type))

        conflicts = {}
        for conn_name, types in name_to_types.items():
            if len(types) > 1:
                modules_by_type = defaultdict(list)
                for module, conn_type in name_to_modules[conn_name]:
                    modules_by_type[conn_type].append(module)
                conflicts[conn_name] = modules_by_type

        if not conflicts:
            return

        error_lines = ["Blueprint cannot start because there are conflicting streams."]
        for name, modules_by_type in conflicts.items():
            type_entries = []
            for conn_type, modules in modules_by_type.items():
                for module in modules:
                    type_str = f"{conn_type.__module__}.{conn_type.__name__}"
                    module_str = module.__name__
                    type_entries.append((type_str, module_str))
            if len(type_entries) >= 2:
                locations = ", ".join(f"{type_} in {module}" for type_, module in type_entries)
                error_lines.append(f"    - '{name}' has conflicting types. {locations}")

        raise ValueError("\n".join(error_lines))

    def _compile_module_specs(self, g: GlobalConfig) -> list[ModuleSpec]:
        """Compile the list of module deployment specs (pure — no side effects)."""
        specs: list[ModuleSpec] = []
        for blueprint in self._active_blueprints:
            specs.append((blueprint.module, g, blueprint.kwargs))
        return specs

    def _compile_stream_wiring(self) -> list[StreamWiring]:
        """Compile stream transport assignments (pure — no side effects)."""
        # Group streams by (remapped_name, type) -> [(module_class, original_name)]
        streams: dict[
            tuple[str | type[ModuleBase] | type[Spec], type], list[tuple[type[ModuleBase], str]]
        ] = defaultdict(list)

        for blueprint in self._active_blueprints:
            for conn in blueprint.streams:
                remapped_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                if isinstance(remapped_name, str):
                    streams[remapped_name, conn.type].append((blueprint.module, conn.name))

        wiring: list[StreamWiring] = []
        for (remapped_name, stream_type), module_streams in streams.items():
            assert isinstance(remapped_name, str)
            transport = self._get_transport_for(remapped_name, stream_type)
            for module_class, original_name in module_streams:
                wiring.append(
                    StreamWiring(
                        module_class=module_class,
                        stream_name=original_name,
                        transport=transport,
                    )
                )
                logger.info(
                    "Transport",
                    name=remapped_name,
                    original_name=original_name,
                    topic=str(getattr(transport, "topic", None)),
                    type=f"{stream_type.__module__}.{stream_type.__qualname__}",
                    module=module_class.__name__,
                    transport=transport.__class__.__name__,
                )
        return wiring

    def _compile_module_ref_wiring(self) -> list[ModuleRefWiring]:
        """Resolve module references and return wiring plan (pure — no side effects)."""
        mod_and_mod_ref_to_target: dict[tuple[type[ModuleBase], str], type[ModuleBase]] = {}

        # Seed with explicit remappings that point to modules/specs
        for (module, name), replacement in self.remapping_map.items():
            if is_module_type(replacement):
                mod_and_mod_ref_to_target[module, name] = replacement  # type: ignore[assignment]

        for blueprint in self._active_blueprints:
            for each_module_ref in blueprint.module_refs:
                key = (blueprint.module, each_module_ref.name)
                if key in mod_and_mod_ref_to_target:
                    continue

                spec = self.remapping_map.get(key, each_module_ref.spec)
                if is_module_type(spec):
                    mod_and_mod_ref_to_target[key] = spec  # type: ignore[assignment]
                    continue

                possible_module_candidates = [
                    each_other_blueprint.module
                    for each_other_blueprint in self._active_blueprints
                    if (
                        each_other_blueprint != blueprint
                        and spec_structural_compliance(each_other_blueprint.module, spec)
                    )
                ]
                valid_module_candidates = [
                    each_candidate
                    for each_candidate in possible_module_candidates
                    if spec_annotation_compliance(each_candidate, spec)
                ]

                if len(possible_module_candidates) == 0:
                    raise Exception(
                        f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. But I couldn't find a module that met that spec.\n"""
                    )
                elif len(possible_module_candidates) == 1:
                    if len(valid_module_candidates) == 0:
                        logger.warning(
                            f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. I found a module ({possible_module_candidates[0].__name__}) that met that spec structurally, but it had a mismatch in type annotations.\nPlease either change the {each_module_ref.spec.__name__} spec or the {possible_module_candidates[0].__name__} module.\n"""
                        )
                    mod_and_mod_ref_to_target[key] = possible_module_candidates[0]
                elif len(valid_module_candidates) > 1:
                    raise Exception(
                        f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. But I found multiple modules that met that spec: {possible_module_candidates}.\nTo fix this use .remappings, for example:\n    autoconnect(...).remappings([ ({blueprint.module.__name__}, {each_module_ref.name!r}, <ModuleThatHasTheRpcCalls>) ])\n"""
                    )
                elif len(valid_module_candidates) == 0:
                    possible_module_candidates_str = ", ".join(
                        [each_candidate.__name__ for each_candidate in possible_module_candidates]
                    )
                    raise Exception(
                        f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. Some modules ({possible_module_candidates_str}) met the spec structurally but had a mismatch in type annotations\n"""
                    )
                else:
                    mod_and_mod_ref_to_target[key] = valid_module_candidates[0]

        return [
            ModuleRefWiring(base_module=base_module, ref_name=ref_name, target_module=target)
            for (base_module, ref_name), target in mod_and_mod_ref_to_target.items()
        ]

    def _compile_rpc_wiring(self) -> RpcWiringPlan:
        """Compile the RPC method registry and binding requests (pure — no side effects)."""
        # registry: rpc_key -> (module_class, method_name)
        registry: dict[str, tuple[type[ModuleBase], str]] = {}

        # Track interface methods to detect ambiguity
        interface_methods: defaultdict[str, list[tuple[type[ModuleBase], str]]] = defaultdict(list)
        interface_methods_dot: defaultdict[str, list[tuple[type[ModuleBase], str]]] = defaultdict(
            list
        )

        for blueprint in self._active_blueprints:
            for method_name in blueprint.module.rpcs.keys():  # type: ignore[attr-defined]
                registry[f"{blueprint.module.__name__}_{method_name}"] = (
                    blueprint.module,
                    method_name,
                )
                registry[f"{blueprint.module.__name__}.{method_name}"] = (
                    blueprint.module,
                    method_name,
                )

                for base in blueprint.module.mro():
                    if (
                        base is not Module
                        and issubclass(base, ABC)
                        and hasattr(base, method_name)
                        and getattr(base, method_name, None) is not None
                    ):
                        interface_methods_dot[f"{base.__name__}.{method_name}"].append(
                            (blueprint.module, method_name)
                        )
                        interface_methods[f"{base.__name__}_{method_name}"].append(
                            (blueprint.module, method_name)
                        )

        # Add non-ambiguous interface methods to registry
        for key, implementations in interface_methods_dot.items():
            if len(implementations) == 1:
                registry[key] = implementations[0]
        for key, implementations in interface_methods.items():
            if len(implementations) == 1:
                registry[key] = implementations[0]

        # Compile set_ method bindings
        set_methods: list[tuple[type[ModuleBase], str, str]] = []
        for blueprint in self._active_blueprints:
            for method_name in blueprint.module.rpcs.keys():  # type: ignore[attr-defined]
                if not method_name.startswith("set_"):
                    continue
                linked_name = method_name.removeprefix("set_")
                self._check_ambiguity(linked_name, interface_methods, blueprint.module)
                if linked_name in registry:
                    set_methods.append((blueprint.module, method_name, linked_name))

        # Compile rpc_call bindings (uses rpc_calls list from module)
        rpc_call_bindings: list[tuple[type[ModuleBase], str, str]] = []
        for blueprint in self._active_blueprints:
            rpc_call_names: list[str] = getattr(blueprint.module, "rpc_calls", [])
            for requested_name in rpc_call_names:
                self._check_ambiguity(requested_name, interface_methods_dot, blueprint.module)
                if requested_name in registry:
                    rpc_call_bindings.append((blueprint.module, requested_name, requested_name))

        return RpcWiringPlan(
            registry=registry,
            set_methods=tuple(set_methods),
            rpc_call_bindings=tuple(rpc_call_bindings),
        )

    def build(
        self,
        cli_config_overrides: Mapping[str, Any] | None = None,
    ) -> ModuleCoordinator:
        logger.info("Building the blueprint")

        # Phase 1: Configuration
        global_config.update(**dict(self.global_config_overrides))
        if cli_config_overrides:
            global_config.update(**dict(cli_config_overrides))

        # Phase 2: Validation
        self._run_configurators()
        self._check_requirements()
        self._verify_no_name_conflicts()

        # Phase 3: Compile wiring plans (pure — no side effects)
        module_specs = self._compile_module_specs(global_config)
        stream_wiring = self._compile_stream_wiring()
        module_ref_wiring = self._compile_module_ref_wiring()
        rpc_wiring = self._compile_rpc_wiring()

        # Phase 4: Execute (all mutations go through coordinator)
        logger.info("Starting the modules")
        coordinator = ModuleCoordinator(g=global_config)
        coordinator.start()
        coordinator.deploy_parallel(module_specs)
        coordinator.wire_streams(stream_wiring)
        coordinator.wire_rpc_methods(rpc_wiring)
        coordinator.wire_module_refs(module_ref_wiring)
        coordinator.build_all_modules()
        coordinator.start_all_modules()

        return coordinator


def autoconnect(*blueprints: Blueprint) -> Blueprint:
    all_blueprints = tuple(_eliminate_duplicates([bp for bs in blueprints for bp in bs.blueprints]))
    all_transports = dict(  # type: ignore[var-annotated]
        reduce(operator.iadd, [list(x.transport_map.items()) for x in blueprints], [])
    )
    all_config_overrides = dict(  # type: ignore[var-annotated]
        reduce(operator.iadd, [list(x.global_config_overrides.items()) for x in blueprints], [])
    )
    all_remappings = dict(  # type: ignore[var-annotated]
        reduce(operator.iadd, [list(x.remapping_map.items()) for x in blueprints], [])
    )
    all_requirement_checks = tuple(check for bs in blueprints for check in bs.requirement_checks)
    all_configurator_checks = tuple(check for bs in blueprints for check in bs.configurator_checks)

    return Blueprint(
        blueprints=all_blueprints,
        disabled_modules_tuple=tuple(
            module for bp in blueprints for module in bp.disabled_modules_tuple
        ),
        transport_map=MappingProxyType(all_transports),
        global_config_overrides=MappingProxyType(all_config_overrides),
        remapping_map=MappingProxyType(all_remappings),
        requirement_checks=all_requirement_checks,
        configurator_checks=all_configurator_checks,
    )


def _eliminate_duplicates(blueprints: list[_BlueprintAtom]) -> list[_BlueprintAtom]:
    seen = set()
    unique_blueprints = []
    for bp in reversed(blueprints):
        if bp.module not in seen:
            seen.add(bp.module)
            unique_blueprints.append(bp)
    return list(reversed(unique_blueprints))
