import * as React from "react";

import Connection from "./Connection";
import ExplorePanel from "./ExplorePanel";
import GpsButton from "./GpsButton";
import Button from "./Button";
import KeyboardControlPanel from "./KeyboardControlPanel";
import VisualizerWrapper from "./components/VisualizerWrapper";
import LeafletMap from "./components/LeafletMap";
import { AppAction, AppState, LatLon } from "./types";

function appReducer(state: AppState, action: AppAction): AppState {
  switch (action.type) {
    case "SET_COSTMAP":
      return { ...state, costmap: action.payload };
    case "SET_ROBOT_POSE":
      return { ...state, robotPose: action.payload };
    case "SET_GPS_LOCATION":
      return { ...state, gpsLocation: action.payload };
    case "SET_GPS_TRAVEL_GOAL_POINTS":
      return { ...state, gpsTravelGoalPoints: action.payload };
    case "SET_PATH":
      return { ...state, path: action.payload };
    case "SET_FULL_STATE":
      return { ...state, ...action.payload };
    default:
      return state;
  }
}

const initialState: AppState = {
  costmap: null,
  robotPose: null,
  gpsLocation: null,
  gpsTravelGoalPoints: null,
  path: null,
};

export default function App(): React.ReactElement {
  const [state, dispatch] = React.useReducer(appReducer, initialState);
  const [isGpsMode, setIsGpsMode] = React.useState(false);
  const connectionRef = React.useRef<Connection | null>(null);

  const [policyEnabled, setPolicyEnabled] = React.useState(false);
  const [policyEstop, setPolicyEstop] = React.useState(false);
  const [policyParams, setPolicyParams] = React.useState(() => ({
    stand: true,
    base_height: 0.75,
    waist_rpy: [0, 0, 0] as [number, number, number],
    // EE targets are offsets from Falcon defaults (meters)
    ee_left_xyz: [0, 0, 0] as [number, number, number],
    ee_right_xyz: [0, 0, 0] as [number, number, number],
    ee_yaw_deg: 0,
    kp_scale: 1.0,
    upper_body_ik_enabled: false,
    upper_body_collision_check: true,
  }));

  React.useEffect(() => {
    connectionRef.current = new Connection(dispatch);

    return () => {
      if (connectionRef.current) {
        connectionRef.current.disconnect();
      }
    };
  }, []);

  React.useEffect(() => {
    const t = window.setTimeout(() => {
      connectionRef.current?.policyParams(policyParams);
    }, 50);
    return () => window.clearTimeout(t);
  }, [policyParams]);

  const handleWorldClick = React.useCallback((worldX: number, worldY: number) => {
    connectionRef.current?.worldClick(worldX, worldY);
  }, []);

  const handleStartExplore = React.useCallback(() => {
    connectionRef.current?.startExplore();
  }, []);

  const handleStopExplore = React.useCallback(() => {
    connectionRef.current?.stopExplore();
  }, []);

  const handleGpsGoal = React.useCallback((goal: LatLon) => {
    connectionRef.current?.sendGpsGoal(goal);
  }, []);

  const handleSendMoveCommand = React.useCallback(
    (linear: [number, number, number], angular: [number, number, number]) => {
      connectionRef.current?.sendMoveCommand(linear, angular);
    },
    [],
  );

  const handleStopMoveCommand = React.useCallback(() => {
    connectionRef.current?.stopMoveCommand();
  }, []);

  const handleReturnHome = React.useCallback(() => {
    connectionRef.current?.worldClick(0, 0);
  }, []);

  const handleStop = React.useCallback(() => {
    if (state.robotPose) {
      connectionRef.current?.worldClick(state.robotPose.coords[0]!, state.robotPose.coords[1]!);
    }
  }, [state.robotPose]);

  const handleTogglePolicyEnable = React.useCallback(() => {
    const nextEnabled = !policyEnabled;
    setPolicyEnabled(nextEnabled);
    connectionRef.current?.safetyCommand(nextEnabled, policyEstop);
  }, [policyEnabled, policyEstop]);

  const handleToggleEstop = React.useCallback(() => {
    const nextEstop = !policyEstop;
    setPolicyEstop(nextEstop);
    // If estop is asserted, force policy disabled as well.
    const nextEnabled = nextEstop ? false : policyEnabled;
    setPolicyEnabled(nextEnabled);
    connectionRef.current?.safetyCommand(nextEnabled, nextEstop);
  }, [policyEnabled, policyEstop]);

  return (
    <div style={{ position: "relative", width: "100%", height: "100%" }}>
      {isGpsMode ? (
        <LeafletMap
          gpsLocation={state.gpsLocation}
          gpsTravelGoalPoints={state.gpsTravelGoalPoints}
          onGpsGoal={handleGpsGoal}
        />
      ) : (
        <VisualizerWrapper data={state} onWorldClick={handleWorldClick} />
      )}
      <div
        style={{
          position: "absolute",
          bottom: 0,
          left: 0,
          display: "flex",
          width: "100%",
          padding: 5,
          gap: 5,
          alignItems: "flex-end",
        }}
      >
        <GpsButton
          onUseGps={() => setIsGpsMode(true)}
          onUseCostmap={() => setIsGpsMode(false)}
        ></GpsButton>
        <ExplorePanel onStartExplore={handleStartExplore} onStopExplore={handleStopExplore} />
        <Button onClick={handleReturnHome} isActive={false}>Go Home</Button>
        <Button onClick={handleStop} isActive={false}>Stop</Button>
        <Button onClick={handleTogglePolicyEnable} isActive={policyEnabled}>
          {policyEnabled ? "Disable Policy" : "Enable Policy"}
        </Button>
        <Button onClick={handleToggleEstop} isActive={policyEstop}>
          {policyEstop ? "Clear E-Stop" : "E-Stop"}
        </Button>
        <div
          style={{
            display: "flex",
            flexDirection: "column",
            gap: 4,
            padding: 6,
            border: "1px solid rgba(255,255,255,0.15)",
            borderRadius: 6,
            minWidth: 260,
          }}
        >
          <div style={{ display: "flex", gap: 6, alignItems: "center" }}>
            <Button
              onClick={() => setPolicyParams((p) => ({ ...p, stand: !p.stand }))}
              isActive={policyParams.stand}
            >
              {policyParams.stand ? "Walk" : "Stand"}
            </Button>
            <Button
              onClick={() =>
                setPolicyParams((p) => ({
                  ...p,
                  upper_body_ik_enabled: !p.upper_body_ik_enabled,
                }))
              }
              isActive={policyParams.upper_body_ik_enabled}
            >
              {policyParams.upper_body_ik_enabled ? "IK On" : "IK Off"}
            </Button>
            <Button
              onClick={() =>
                setPolicyParams((p) => ({
                  ...p,
                  upper_body_collision_check: !p.upper_body_collision_check,
                }))
              }
              isActive={policyParams.upper_body_collision_check}
            >
              {policyParams.upper_body_collision_check ? "Coll On" : "Coll Off"}
            </Button>
          </div>

          <label style={{ fontSize: 12 }}>
            Base height: {policyParams.base_height.toFixed(2)}
            <input
              type="range"
              min={0.55}
              max={0.90}
              step={0.01}
              value={policyParams.base_height}
              onChange={(e) => setPolicyParams((p) => ({ ...p, base_height: Number(e.target.value) }))}
              style={{ width: "100%" }}
            />
          </label>

          <label style={{ fontSize: 12 }}>
            KP scale: {policyParams.kp_scale.toFixed(2)}
            <input
              type="range"
              min={0.2}
              max={1.5}
              step={0.05}
              value={policyParams.kp_scale}
              onChange={(e) => setPolicyParams((p) => ({ ...p, kp_scale: Number(e.target.value) }))}
              style={{ width: "100%" }}
            />
          </label>

          <label style={{ fontSize: 12 }}>
            Waist yaw: {policyParams.waist_rpy[0].toFixed(2)}
            <input
              type="range"
              min={-1.2}
              max={1.2}
              step={0.05}
              value={policyParams.waist_rpy[0]}
              onChange={(e) =>
                setPolicyParams((p) => ({
                  ...p,
                  waist_rpy: [Number(e.target.value), p.waist_rpy[1], p.waist_rpy[2]],
                }))
              }
              style={{ width: "100%" }}
            />
          </label>

          <label style={{ fontSize: 12 }}>
            Waist pitch: {policyParams.waist_rpy[2].toFixed(2)}
            <input
              type="range"
              min={-1.0}
              max={1.0}
              step={0.05}
              value={policyParams.waist_rpy[2]}
              onChange={(e) =>
                setPolicyParams((p) => ({
                  ...p,
                  waist_rpy: [p.waist_rpy[0], p.waist_rpy[1], Number(e.target.value)],
                }))
              }
              style={{ width: "100%" }}
            />
          </label>

          <label style={{ fontSize: 12 }}>
            EE yaw deg: {policyParams.ee_yaw_deg.toFixed(0)}
            <input
              type="range"
              min={-45}
              max={45}
              step={5}
              value={policyParams.ee_yaw_deg}
              onChange={(e) => setPolicyParams((p) => ({ ...p, ee_yaw_deg: Number(e.target.value) }))}
              style={{ width: "100%" }}
            />
          </label>

          <label style={{ fontSize: 12 }}>
            EE x offset: {policyParams.ee_left_xyz[0].toFixed(2)}
            <input
              type="range"
              min={-0.20}
              max={0.20}
              step={0.01}
              value={policyParams.ee_left_xyz[0]}
              onChange={(e) => {
                const v = Number(e.target.value);
                setPolicyParams((p) => ({
                  ...p,
                  ee_left_xyz: [v, p.ee_left_xyz[1], p.ee_left_xyz[2]],
                  ee_right_xyz: [v, p.ee_right_xyz[1], p.ee_right_xyz[2]],
                }));
              }}
              style={{ width: "100%" }}
            />
          </label>

          <label style={{ fontSize: 12 }}>
            EE y offset: {policyParams.ee_left_xyz[1].toFixed(2)}
            <input
              type="range"
              min={-0.20}
              max={0.20}
              step={0.01}
              value={policyParams.ee_left_xyz[1]}
              onChange={(e) => {
                const v = Number(e.target.value);
                setPolicyParams((p) => ({
                  ...p,
                  ee_left_xyz: [p.ee_left_xyz[0], v, p.ee_left_xyz[2]],
                  ee_right_xyz: [p.ee_right_xyz[0], -v, p.ee_right_xyz[2]],
                }));
              }}
              style={{ width: "100%" }}
            />
          </label>

          <label style={{ fontSize: 12 }}>
            EE z offset: {policyParams.ee_left_xyz[2].toFixed(2)}
            <input
              type="range"
              min={-0.20}
              max={0.20}
              step={0.01}
              value={policyParams.ee_left_xyz[2]}
              onChange={(e) => {
                const v = Number(e.target.value);
                setPolicyParams((p) => ({
                  ...p,
                  ee_left_xyz: [p.ee_left_xyz[0], p.ee_left_xyz[1], v],
                  ee_right_xyz: [p.ee_right_xyz[0], p.ee_right_xyz[1], v],
                }));
              }}
              style={{ width: "100%" }}
            />
          </label>
        </div>
        <KeyboardControlPanel
          onSendMoveCommand={handleSendMoveCommand}
          onStopMoveCommand={handleStopMoveCommand}
        />
      </div>
    </div>
  );
}
