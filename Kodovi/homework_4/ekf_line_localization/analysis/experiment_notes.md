# Homework 4 Experiment Notes

Use identical waypoints for all runs:

```text
config/waypoints.yaml
```

Recommended runs:

```text
a_odom_feedback
b_ekf_feedback_corrected
c_ekf_feedback_prediction_only
b_gate_2
b_gate_4
```

Record topics:

```text
/odom
/ekf_pose
/cmd_vel
/scan
/tf
/tf_static
/gazebo/model_states
/ekf_association_count
/ekf_update_applied
/waypoint_index
```

Homework 3 line extraction was re-run on the maze world for a quick map sanity
check. The captured output is saved in `analysis/homework_3_line_capture.md`.

Report points:

- Odom feedback and prediction-only EKF both drift because no external
  measurement corrects accumulated wheel integration error.
- Corrected EKF should reduce long-term pose error when line associations are
  accepted.
- EKF covariance grows during prediction and drops after accepted line updates.
- Correction jumps can create abrupt controller errors; velocity saturation and
  rate limiting bound the resulting `/cmd_vel` spikes.
- A small validation gate rejects useful lines; a large gate accepts more lines
  but increases the risk of wrong associations.
