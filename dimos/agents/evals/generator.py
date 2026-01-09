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

"""Eval generator module for creating fine-tuning datasets from DIMOS blueprints."""

from datetime import datetime
import json
from pathlib import Path
from typing import Any
import uuid

from langchain.chat_models import init_chat_model
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import HumanMessage, SystemMessage

from dimos.agents.evals.config import EvalGeneratorConfig
from dimos.agents.evals.prompts import (
    MULTI_TURN_SYSTEM_PROMPT,
    SINGLE_TURN_SYSTEM_PROMPT,
    build_multi_turn_prompt,
    build_single_turn_prompt,
)
from dimos.agents.evals.schema_extractor import extract_skills_from_blueprint
from dimos.agents.spec import ToolSchemaList
from dimos.core.blueprints import ModuleBlueprintSet
from dimos.core.module import Module
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class EvalGenerator(Module):
    """Module for generating fine-tuning evaluation datasets from blueprints.

    Example:
        ```python
        from dimos.agents.eval import EvalGenerator, EvalGeneratorConfig
        from dimos.robot.all_blueprints import get_blueprint_by_name

        blueprint = get_blueprint_by_name('unitree-go2-agentic')
        gen = EvalGenerator(num_evals=100)
        jsonl_path, json_path = gen.generate_from_blueprint(blueprint)
        ```
    """

    default_config: type[EvalGeneratorConfig] = EvalGeneratorConfig
    config: EvalGeneratorConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._llm: BaseChatModel | None = None

    @property
    def llm(self) -> BaseChatModel:
        """Lazily initialize the LLM."""
        if self._llm is None:
            # Handle both enum and string types for model/provider
            model = (
                self.config.model.value
                if hasattr(self.config.model, "value")
                else self.config.model
            )
            provider = (
                self.config.provider.value
                if hasattr(self.config.provider, "value")
                else self.config.provider
            )
            self._llm = init_chat_model(
                model_provider=provider,
                model=model,
                temperature=self.config.temperature,
            )
        return self._llm

    def generate_from_blueprint(
        self,
        blueprint: ModuleBlueprintSet,
        output_prefix: str = "evals",
    ) -> tuple[Path | None, Path | None]:
        """Generate evaluation dataset from a blueprint.

        Args:
            blueprint: The DIMOS blueprint to extract skills from.
            output_prefix: Prefix for output filenames.

        Returns:
            Tuple of (jsonl_path, json_path), either can be None based on config.
        """
        # Extract skills from the blueprint
        tools = extract_skills_from_blueprint(blueprint)
        if not tools:
            logger.warning("No skills found in blueprint")
            return None, None

        logger.info(f"Found {len(tools)} tools in blueprint")

        # Generate evals
        all_evals: list[dict[str, Any]] = []

        if self.config.include_single_turn:
            # Ensure at least 1 eval when both types enabled
            if self.config.include_multi_turn:
                single_turn_count = max(1, self.config.num_evals // 2)
            else:
                single_turn_count = self.config.num_evals
            single_turn_evals = self._generate_single_turn_evals(tools, single_turn_count)
            all_evals.extend(single_turn_evals)
            logger.info(f"Generated {len(single_turn_evals)} single-turn evals")

        if self.config.include_multi_turn:
            # Ensure at least 1 eval when both types enabled
            if self.config.include_single_turn:
                multi_turn_count = max(1, self.config.num_evals - len(all_evals))
            else:
                multi_turn_count = self.config.num_evals
            multi_turn_evals = self._generate_multi_turn_evals(
                tools, multi_turn_count, self.config.max_turns_per_conversation
            )
            all_evals.extend(multi_turn_evals)
            logger.info(f"Generated {len(multi_turn_evals)} multi-turn evals")

        # Write output files
        jsonl_path = None
        json_path = None

        self.config.output_dir.mkdir(parents=True, exist_ok=True)

        if self.config.output_format in ("jsonl", "both"):
            jsonl_path = self._write_jsonl(all_evals, output_prefix)

        if self.config.output_format in ("json", "both"):
            json_path = self._write_json(all_evals, tools, output_prefix)

        return jsonl_path, json_path

    def generate_from_tools(
        self,
        tools: ToolSchemaList,
        output_prefix: str = "evals",
    ) -> tuple[Path | None, Path | None]:
        """Generate evaluation dataset from a list of tool definitions.

        Args:
            tools: List of OpenAI-format tool definitions.
            output_prefix: Prefix for output filenames.

        Returns:
            Tuple of (jsonl_path, json_path), either can be None based on config.
        """
        if not tools:
            logger.warning("No tools provided")
            return None, None

        logger.info(f"Generating evals for {len(tools)} tools")

        # Generate evals
        all_evals: list[dict[str, Any]] = []

        if self.config.include_single_turn:
            # Ensure at least 1 eval when both types enabled
            if self.config.include_multi_turn:
                single_turn_count = max(1, self.config.num_evals // 2)
            else:
                single_turn_count = self.config.num_evals
            single_turn_evals = self._generate_single_turn_evals(tools, single_turn_count)
            all_evals.extend(single_turn_evals)

        if self.config.include_multi_turn:
            # Ensure at least 1 eval when both types enabled
            if self.config.include_single_turn:
                multi_turn_count = max(1, self.config.num_evals - len(all_evals))
            else:
                multi_turn_count = self.config.num_evals
            multi_turn_evals = self._generate_multi_turn_evals(
                tools, multi_turn_count, self.config.max_turns_per_conversation
            )
            all_evals.extend(multi_turn_evals)

        # Write output files
        jsonl_path = None
        json_path = None

        self.config.output_dir.mkdir(parents=True, exist_ok=True)

        if self.config.output_format in ("jsonl", "both"):
            jsonl_path = self._write_jsonl(all_evals, output_prefix)

        if self.config.output_format in ("json", "both"):
            json_path = self._write_json(all_evals, tools, output_prefix)

        return jsonl_path, json_path

    def _generate_single_turn_evals(
        self,
        tools: ToolSchemaList,
        num_evals: int,
    ) -> list[dict[str, Any]]:
        """Generate single-turn evaluation examples.

        Args:
            tools: List of OpenAI-format tool definitions.
            num_evals: Number of examples to generate.

        Returns:
            List of eval examples in OpenAI fine-tuning format.
        """
        evals: list[dict[str, Any]] = []
        remaining = num_evals

        while remaining > 0:
            batch_size = min(self.config.batch_size, remaining)

            system_prompt = self.config.generation_prompt or SINGLE_TURN_SYSTEM_PROMPT
            user_prompt = build_single_turn_prompt(tools, batch_size)

            messages = [
                SystemMessage(content=system_prompt),
                HumanMessage(content=user_prompt),
            ]

            try:
                response = self.llm.invoke(messages)
                content = response.content if hasattr(response, "content") else str(response)

                # Parse JSON from response
                generated = self._parse_json_response(content)

                for item in generated:
                    eval_example = self._format_single_turn_eval(item, tools)
                    if eval_example:
                        evals.append(eval_example)

                remaining -= len(generated)
                logger.debug(f"Generated {len(generated)} single-turn evals, {remaining} remaining")

            except Exception as e:
                logger.error(f"Error generating single-turn evals: {e}")
                remaining -= batch_size  # Avoid infinite loop

        return evals[:num_evals]

    def _generate_multi_turn_evals(
        self,
        tools: ToolSchemaList,
        num_evals: int,
        max_turns: int,
    ) -> list[dict[str, Any]]:
        """Generate multi-turn conversation evaluation examples.

        Args:
            tools: List of OpenAI-format tool definitions.
            num_evals: Number of conversations to generate.
            max_turns: Maximum turns per conversation.

        Returns:
            List of eval examples in OpenAI fine-tuning format.
        """
        evals: list[dict[str, Any]] = []
        remaining = num_evals

        while remaining > 0:
            batch_size = min(self.config.batch_size, remaining)

            system_prompt = MULTI_TURN_SYSTEM_PROMPT
            user_prompt = build_multi_turn_prompt(tools, batch_size, max_turns)

            messages = [
                SystemMessage(content=system_prompt),
                HumanMessage(content=user_prompt),
            ]

            try:
                response = self.llm.invoke(messages)
                content = response.content if hasattr(response, "content") else str(response)

                # Parse JSON from response
                generated = self._parse_json_response(content)

                for item in generated:
                    eval_example = self._format_multi_turn_eval(item, tools)
                    if eval_example:
                        evals.append(eval_example)

                remaining -= len(generated)
                logger.debug(f"Generated {len(generated)} multi-turn evals, {remaining} remaining")

            except Exception as e:
                logger.error(f"Error generating multi-turn evals: {e}")
                remaining -= batch_size  # Avoid infinite loop

        return evals[:num_evals]

    def _parse_json_response(self, content: str) -> list[dict[str, Any]]:
        """Parse JSON array from LLM response.

        Args:
            content: The raw LLM response content.

        Returns:
            Parsed list of dictionaries.
        """
        # Try to extract JSON from the response
        content = content.strip()

        # Handle markdown code blocks
        if "```json" in content:
            start = content.find("```json") + 7
            end = content.find("```", start)
            content = content[start:end].strip()
        elif "```" in content:
            start = content.find("```") + 3
            end = content.find("```", start)
            content = content[start:end].strip()

        # Find JSON array bounds
        if "[" in content:
            start = content.find("[")
            end = content.rfind("]") + 1
            content = content[start:end]

        try:
            result = json.loads(content)
            if isinstance(result, list):
                return result
            return [result]
        except json.JSONDecodeError as e:
            logger.warning(f"Failed to parse JSON response: {e}")
            return []

    def _format_single_turn_eval(
        self,
        item: dict[str, Any],
        tools: ToolSchemaList,
    ) -> dict[str, Any] | None:
        """Format a single-turn eval into OpenAI fine-tuning format.

        Args:
            item: Generated item with user_query and tool_calls.
            tools: Available tool definitions for system message.

        Returns:
            Formatted eval or None if invalid.
        """
        user_query = item.get("user_query", "")
        tool_calls = item.get("tool_calls", [])

        if not user_query or not tool_calls:
            return None

        # Build the messages array
        messages: list[dict[str, Any]] = [
            {
                "role": "system",
                "content": "You are a helpful robot assistant. Use the available tools to help the user.",
            },
            {"role": "user", "content": user_query},
        ]

        # Format assistant message with tool calls
        assistant_msg: dict[str, Any] = {"role": "assistant"}
        formatted_tool_calls = []

        for tc in tool_calls:
            tool_call = {
                "id": f"call_{uuid.uuid4().hex[:8]}",
                "type": "function",
                "function": {
                    "name": tc.get("name", ""),
                    "arguments": json.dumps(tc.get("arguments", {})),
                },
            }
            formatted_tool_calls.append(tool_call)

        if formatted_tool_calls:
            assistant_msg["tool_calls"] = formatted_tool_calls

        messages.append(assistant_msg)

        return {"messages": messages, "tools": tools}

    def _format_multi_turn_eval(
        self,
        item: dict[str, Any],
        tools: ToolSchemaList,
    ) -> dict[str, Any] | None:
        """Format a multi-turn eval into OpenAI fine-tuning format.

        Args:
            item: Generated conversation item.
            tools: Available tool definitions.

        Returns:
            Formatted eval or None if invalid.
        """
        conversation = item.get("conversation", [])
        if not conversation:
            return None

        messages: list[dict[str, Any]] = [
            {
                "role": "system",
                "content": "You are a helpful robot assistant. Use the available tools to help the user.",
            }
        ]

        for msg in conversation:
            role = msg.get("role", "")
            content = msg.get("content", "")

            if role == "user":
                messages.append({"role": "user", "content": content})

            elif role == "assistant":
                assistant_msg: dict[str, Any] = {"role": "assistant"}
                if content:
                    assistant_msg["content"] = content

                tool_calls = msg.get("tool_calls", [])
                if tool_calls:
                    formatted_tool_calls = []
                    for tc in tool_calls:
                        tool_call = {
                            "id": f"call_{uuid.uuid4().hex[:8]}",
                            "type": "function",
                            "function": {
                                "name": tc.get("name", ""),
                                "arguments": json.dumps(tc.get("arguments", {})),
                            },
                        }
                        formatted_tool_calls.append(tool_call)
                    assistant_msg["tool_calls"] = formatted_tool_calls

                messages.append(assistant_msg)

            elif role == "tool_result":
                # Format as tool message
                messages.append(
                    {
                        "role": "tool",
                        "tool_call_id": f"call_{uuid.uuid4().hex[:8]}",
                        "name": msg.get("name", ""),
                        "content": content,
                    }
                )

        return {"messages": messages, "tools": tools}

    def _write_jsonl(
        self,
        evals: list[dict[str, Any]],
        output_prefix: str,
    ) -> Path:
        """Write evals to JSONL file for OpenAI fine-tuning.

        Args:
            evals: List of eval examples.
            output_prefix: Filename prefix.

        Returns:
            Path to the written file.
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{output_prefix}_{timestamp}.jsonl"
        filepath = self.config.output_dir / filename

        with open(filepath, "w") as f:
            for eval_item in evals:
                # JSONL format: just the messages, tools go in the fine-tuning config
                jsonl_item = {"messages": eval_item.get("messages", [])}
                f.write(json.dumps(jsonl_item) + "\n")

        logger.info(f"Wrote {len(evals)} evals to {filepath}")
        return filepath

    def _write_json(
        self,
        evals: list[dict[str, Any]],
        tools: ToolSchemaList,
        output_prefix: str,
    ) -> Path:
        """Write evals to JSON file for inspection.

        Args:
            evals: List of eval examples.
            tools: Tool definitions used.
            output_prefix: Filename prefix.

        Returns:
            Path to the written file.
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{output_prefix}_{timestamp}.json"
        filepath = self.config.output_dir / filename

        output = {
            "metadata": {
                "generated_at": datetime.now().isoformat(),
                "num_evals": len(evals),
                "model": self.config.model,
                "provider": self.config.provider,
                "tools": tools,
            },
            "evals": evals,
        }

        with open(filepath, "w") as f:
            json.dump(output, f, indent=2)

        logger.info(f"Wrote {len(evals)} evals to {filepath}")
        return filepath

    def start(self) -> None:
        """Start the module."""
        super().start()

    def stop(self) -> None:
        """Stop the module."""
        self._llm = None
        super().stop()


# Blueprint for use in DIMOS pipelines
eval_generator = EvalGenerator.blueprint

__all__ = ["EvalGenerator", "eval_generator"]
