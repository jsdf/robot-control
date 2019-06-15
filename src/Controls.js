// @flow

import type IKRobot from './IKRobot';
import React from 'react';
import ReactDOM from 'react-dom';

function Controls(props: {robot: IKRobot}) {
  return (
    <div style={{position: 'absolute', top: 0, right: 0}}>
      <button
        onClick={e => {
          if (props.robot.plannedArmSolution.solutionIsValid()) {
            props.robot.commitPlan();
          }
        }}
      >
        commit
      </button>
    </div>
  );
}

export default function renderControls(node: Element, robot: IKRobot) {
  ReactDOM.render(<Controls robot={robot} />, node);
}
