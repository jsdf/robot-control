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

function Controls(props: {robot: IKRobot}) {
  const [serverResponse, setServerResponse] = React.useState(null);
  const [keyframes, setKeyframes] = React.useState([]);
  const [loop, setLoop] = React.useState(true);

  return (
    <div>
      <div style={{position: 'absolute', top: 0, right: 0}}>
        <button
          onClick={e => {
            if (props.robot.plannedArmSolution.solutionIsValid()) {
              const plan = props.robot.commitPlan();
              sentToServer(plan).then(res => {
                console.log(res);
                if (res) {
                  setServerResponse(res.output);
                }
              });
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
        <pre>{JSON.stringify(serverResponse, null, 2)}</pre>
      </div>

      <div style={{position: 'absolute', bottom: 0, left: 0}}>
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
    </div>
  );
}

export default function renderControls(node: Element, robot: IKRobot) {
  ReactDOM.render(<Controls robot={robot} />, node);
}
