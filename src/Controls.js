// @flow

import type IKRobot from './IKRobot';
import React from 'react';
import ReactDOM from 'react-dom';

async function sentToServer(plan: Array<number>) {
  console.log('committing', plan);
  const host = '192.168.7.45';

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
    debugger;
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
function CommitControls(props: {robot: IKRobot}) {
  const [serverResponse, setServerResponse] = React.useState(null);
  const [clawAngle, setClawAngle] = React.useState(0);
  const [clawGrip, setClawGrip] = React.useState(0);

  async function commitToServer(plan, clawAngle, clawGrip) {
    const res = await sentToServer(plan.concat(clawAngle, clawGrip));

    console.log(res);
    if (res) {
      setServerResponse(res.output);
    }
  }

  return (
    <div style={{textAlign: 'right'}}>
      <div>
        <button
          onClick={e => {
            if (props.robot.plannedArmSolution.solutionIsValid()) {
              const plan = props.robot.commitPlan();
              commitToServer(plan, clawAngle, clawGrip);
            }
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
              commitToServer(props.robot.getCommitted(), updated, clawGrip);
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
              commitToServer(props.robot.getCommitted(), clawAngle, updated);
            }}
          />
        </label>
      </div>
      <pre>{JSON.stringify(serverResponse, null, 2)}</pre>
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
