R/W is from perspective of HMI, not CC

# Coils

| Index | Var name                        | R/W (from HMI) | Description                                                                                                                                               |
|-------|---------------------------------|----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0     | is_mandrel_latch_closed         | R              | mandrel latch sensor reading. True for closed and safe                                                                                                    |
| 1     | is_fingers_down                 | R              | Are the workpeice holding fingers commanded down                                                                                                          |
| 2     | is_homed                        | R              | Has the axis been homed                                                                                                                                   |
| 3     | is_fault                        | R              | Not sure what, but the software has reached one of those `// this should never happen` comments. HMI should show that there is a problem with the CC      |
| 4     | is_ready_for_cycle              | R              | Is the clearcore ready to execute it's program                                                                                                            |
| 5     | is_e_stop                       | R              | Is the E-stop currently active                                                                                                                            |
| 6     | is_job_active                   | R              | Is the clearcore executing it's program                                                                                                                   |
| 7     | is_ready_for_manual_control     | R              | Is the clearcore ready to take a commanded position. this is safety checks as well as making sure there is no cycle in progress                           |
| 8     | is_roller_down                  | R              | Is the roller currently down/engaged                                                                                                                      |
| 20    | is_commanded_pos                | R/W            | Has the HMI requested a new commanded position? If true it should read the commanded pos and go there and then unlatch, else the commanded pos is ignored |
| 30    | is_rth_button_latched           | R/W            | Has the 'return to home' button been pressed since last check? Gets set back to 0 on completion                                                           |
| 31    | is_axis_homing_button_latched   | R/W            | Has the 'run axis homing sequence' button been pressed since last check? Gets set back to 0 on completion                                                 |
| 32    | is_set_job_start_button_latched | R/W            | Has the 'Set start to current position' button been pressed since last check? Gets set back to 0 on completion                                            |
| 33    | is_set_job_end_button_latched   | R/W            | Has the 'Set end to current position' button been pressed since last check? Gets set back to 0 on completion                                              |
| 34    | is_set_job_park_button_latched  | R/W            | Has the 'Set park to current position' button been pressed since last check? Gets set back to 0 on completion                                             |
| 40    | commanded_fingers               | R/W            | Finger state commanded by the HMI, true means engaged, false means disengaged                                                                             |
| 41    | commanded_roller                | R/W            | roller state commanded by the HMI, true means engaged, false means disengaged                                                                             |



# Hregs

ALL u16

| Index | Var name           | R/W (from HMI) | Description                                                                                                                                                      |
|-------|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2     | actual_position    | R              | actual position of the axis measured in hundreths of an inch                                                                                                     |
| 12    | commanded_position | R/W            | commanded position of the axis measured in hundreths of an inch. The commanded pos will not be acted upon if is_commanded_pos is not set                         |
| 16    | job_progress       | R              | The progress of the current job if there is one. (job_progress/65536)*100%                                                                                       |
| 19    | job_start_pos      | R              | The currently saved start position of the axis measured in hundreths of an inch. (While the register is read only, the value can semantically changed otherwise) |
| 21    | job_end_pos        | R              | Same as job_start_pos_upper but for end                                                                                                                          |
| 23    | job_park_pos       | R              | Same as job_start_pos_upper but for park                                                                                                                         |
| 25    | min_pos            | R              | minimum absolute position of the axis measured in hundreths of an inch.                                                                                          |
| 26    | max_pos            | R              | maximum absolute position of the axis measured in hundreths of an inch.                                                                                          |
| 32    | jog_speed          | R/W            | target speed while jogging manually or in disengaged portion of job sequence                                                                                     |
| 33    | planish_speed      | R/W            | target speed while in engaged portion of job sequence                                                                                                            |
| 40    | fault_code         | R              | Fault code. 0 is normal operation                                                                                                                                |
| 50    | heartbeat_in       | W              | Contains an arbitrary value set by the HMI                                                                                                                       |
| 51    | heartbeat_out      | R              | This should always be equal to `(heartbeat_in*2)%65536`                                                                                                          |