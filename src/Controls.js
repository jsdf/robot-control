// @flow

import type IKRobot from './IKRobot';
import React from 'react';
import ReactDOM from 'react-dom';

function sentToServer(plan: Array<number>) {
  console.log('committing', plan);

  // return fetch('http://raspberrypi.local:8080/move', {
  //   method: 'POST',
  //   mode: 'cors',
  //   headers: {
  //     'Content-Type': 'application/json',
  //   },
  //   body: JSON.stringify({plan}),
  // }).then(response => response.json()); // parses JSON response into native Javascript objects
}

function Controls(props: {robot: IKRobot}) {
  return (
    <div style={{position: 'absolute', top: 0, right: 0}}>
      <button
        onClick={e => {
          if (props.robot.plannedArmSolution.solutionIsValid()) {
            const plan = props.robot.commitPlan();
            sentToServer(plan);
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
        reset
      </button>
    </div>
  );
}

export default function renderControls(node: Element, robot: IKRobot) {
  ReactDOM.render(<Controls robot={robot} />, node);
}
