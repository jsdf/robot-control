// @flow

import type IKRobot from './IKRobot';
import React from 'react';
import ReactDOM from 'react-dom';

const ARM_JOINTS = {
  rotate_base: 0,
  arm_bottom_joint: 1,
  arm_middle_joint: 2,
  tilt_claw: 3,
  rotate_claw: 4,
  close_claw: 5,
};

const defaultHost = '192.168.7.45';

async function checkOnline(host) {
  try {
    await fetch(`http://${host}:8080/health`);
    return true;
  } catch (err) {
    return false;
  }
}

function degToRad(angleDegrees: number): number {
  return angleDegrees * (Math.PI / 180.0);
}

async function sendToServer(host: string, plan: Array<number>) {
  console.log('committing', plan);

  try {
    const response = await fetch(
      `http://${host}:8080/move?` + JSON.stringify({plan}),
      {
        method: 'GET',
        mode: 'cors',
      }
    );
    return await response.json();
  } catch (err) {
    console.error(err);
  }
}

function FrameCount(props: {robot: IKRobot}) {
  const [currentFrame, setCurrentFrame] = React.useState(0);

  React.useEffect(() => {
    let interval = setInterval(() => {
      const {animation} = props.robot;
      setCurrentFrame(animation ? animation.currentFrame : 0);
    });

    return () => clearInterval(interval);
  });

  return <span>keyframe: {currentFrame}</span>;
}

function AnimControls(props: {robot: IKRobot}) {
  const [keyframes, setKeyframes] = React.useState([]);
  const [loop, setLoop] = React.useState(true);

  return (
    <div>
      <button
        onClick={() => {
          setKeyframes([]);
        }}
      >
        clear
      </button>
      <button
        onClick={() => {
          if (props.robot.plannedArmSolution.solutionIsValid()) {
            const plan = props.robot.getPlan();
            setKeyframes(
              keyframes.concat({
                plan,
                interval: 1,
              })
            );
          }
        }}
      >
        add keyframe
      </button>
      <button
        onClick={() => {
          props.robot.playAnimation(keyframes, loop);
        }}
      >
        play
      </button>
      <label>
        loop:
        <input
          type="checkbox"
          checked={loop}
          onChange={event => {
            setLoop(loop => !loop);
          }}
        />
      </label>
      <FrameCount robot={props.robot} />
      <div style={{height: 30}}>
        {keyframes.map((frame, i) => (
          <span key={i}>
            <button
              onClick={() => {
                props.robot.loadPlan(frame.plan);
              }}
            >
              {i}
            </button>
            {false && (
              <input
                type="number"
                value={frame.interval}
                onChange={event => {
                  setKeyframes(
                    keyframes.map((frame, keyframeIndex) => {
                      if (i === keyframeIndex) {
                        return {
                          ...frame,
                          interval: parseFloat(event.target.value),
                        };
                      }
                      return frame;
                    })
                  );
                }}
              />
            )}
          </span>
        ))}
      </div>
    </div>
  );
}

function within(a, b, distance) {
  return Math.abs(a - b) < 0.001;
}

function CommitControls(props: {robot: IKRobot}) {
  const [serverResponse, setServerResponse] = React.useState(null);
  const [clawAngle, setClawAngle] = React.useState(0);
  const [clawGrip, setClawGrip] = React.useState(0);
  const [network, setNetwork] = React.useState({
    host: defaultHost,
    online: false,
    enabled: true,
  });

  function checkAndUpdateNetwork(updatedHost = network.host) {
    checkOnline(updatedHost).then(online => {
      setNetwork(prev =>
        prev.host === updatedHost && prev.online != online
          ? {...prev, online}
          : prev
      );
    });
  }

  React.useEffect(() => {
    console.log('checkAndUpdateNetwork from useEffect');
    checkAndUpdateNetwork();
  }, []);

  const commitedRef = React.useRef(null);

  async function commitToServer(plan, clawAngle, clawGrip) {
    const fullArmConfiguration: Array<number> = [
      plan[ARM_JOINTS.rotate_base],
      plan[ARM_JOINTS.arm_bottom_joint],
      plan[ARM_JOINTS.arm_middle_joint],
      plan[ARM_JOINTS.tilt_claw],
      clawAngle,
      clawGrip,
    ];

    const prev = commitedRef.current;
    if (
      prev &&
      fullArmConfiguration.every((val, i) => within(prev[i], val, 0.0001))
    ) {
      return;
    }

    commitedRef.current = fullArmConfiguration;

    const configWithNames = {};
    Object.keys(ARM_JOINTS).forEach((key, i) => {
      configWithNames[key] = fullArmConfiguration[i];
    });

    console.log('committing', configWithNames);

    if (!network.online) {
      console.log('offline, skipping request to server');
      return;
    }
    if (!network.enabled) {
      console.log('network disabled, skipping request to server');
      return;
    }

    const res = await sendToServer(network.host, fullArmConfiguration);

    console.log(res);
    if (res) {
      setServerResponse(res.output);
    }
  }

  function commitIfValid() {
    if (props.robot.plannedArmSolution.solutionIsValid()) {
      const plan = props.robot.commitPlan();
      commitToServer(plan, degToRad(clawAngle), degToRad(clawGrip));
    }
  }

  const [autoCommit, setAutoCommit] = React.useState(false);
  const autoCommitRef = React.useRef(() => {});
  autoCommitRef.current = autoCommit ? commitIfValid : () => {};

  React.useEffect(() => {
    const interval = setInterval(() => {
      if (autoCommitRef.current) {
        autoCommitRef.current();
      }
    }, 10);

    return () => clearInterval(interval);
  }, []);

  return (
    <div style={{textAlign: 'right'}}>
      <div>
        <label>
          enabled:
          <input
            type="checkbox"
            checked={network.enabled}
            onChange={event => {
              setNetwork(prev => ({...prev, enabled: !network.enabled}));
            }}
          />
        </label>
        <label>
          auto-commit:
          <input
            type="checkbox"
            checked={autoCommit}
            onChange={event => {
              setAutoCommit(!autoCommit);
            }}
          />
        </label>
      </div>
      <div>
        {network.online ? 'online' : 'offline'}
        <input
          type="text"
          value={network.host}
          onChange={e => {
            const updated = e.target.value;
            setNetwork(prev => ({...prev, host: updated}));
            console.log('checkAndUpdateNetwork from onChange');
            checkAndUpdateNetwork(updated);
          }}
        />
      </div>
      <div>
        <button
          onClick={e => {
            commitIfValid();
          }}
        >
          commit
        </button>
        <button
          onClick={e => {
            props.robot.resetToInitial();
          }}
        >
          re-solve IK
        </button>
        <button
          onClick={e => {
            props.robot.resetToInitial(true);
          }}
        >
          reset
        </button>
      </div>
      <div>
        <label>
          claw angle <input size={3} readOnly value={Math.floor(clawAngle)} />
          <input
            type="range"
            min={-90}
            max={90}
            width={100}
            value={clawAngle}
            onChange={e => {
              const updated = parseInt(e.target.value);
              setClawAngle(updated);
              commitToServer(
                props.robot.getCommitted(),
                degToRad(updated),
                degToRad(clawGrip)
              );
            }}
          />
        </label>
      </div>
      <div>
        <label>
          claw grip <input size={3} readOnly value={Math.floor(clawGrip)} />
          <input
            type="range"
            min={-90}
            max={90}
            width={100}
            value={clawGrip}
            onChange={e => {
              const updated = parseInt(e.target.value);
              setClawGrip(updated);
              commitToServer(
                props.robot.getCommitted(),
                degToRad(clawAngle),
                degToRad(updated)
              );
            }}
          />
        </label>
      </div>
      <pre style={{textAlign: 'initial'}}>
        servos:
        {'\n' + JSON.stringify(serverResponse || 'no data', null, 2)}
      </pre>
    </div>
  );
}

function Controls(props: {robot: IKRobot}) {
  return (
    <div>
      <div style={{position: 'absolute', top: 0, right: 0}}>
        <CommitControls robot={props.robot} />
      </div>
      <div style={{position: 'absolute', bottom: 0, left: 0}}>
        <AnimControls robot={props.robot} />
      </div>
    </div>
  );
}

export default function renderControls(node: Element, robot: IKRobot) {
  ReactDOM.render(<Controls robot={robot} />, node);
}
