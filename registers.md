R/W is from perspective of HMI, not CC

# Coils

| Index | Var name                        | R/W | Description                                                                                                                                          |
|-------|---------------------------------|-----|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0     | is_mandrel_latch_closed         | R   | mandrel latch sensor reading. True for closed and safe                                                                                               |
| 1     | is_fingers_down                 | R   | Are the workpeice holding fingers commanded down                                                                                                     |
| 2     | is_homed                        | R   | Has the axis been homed                                                                                                                              |
| 3     | is_fault                        | R   | Not sure what, but the software has reached one of those `// this should never happen` comments. HMI should show that there is a problem with the CC |
| 4     | is_ready_for_cycle              | R   | Is the clearcore ready to execute it's program                                                                                                       |
| 5     | is_e_stop                       | R   | Is the E-stop currently active                                                                                                                       |
| 6     | is_job_active                   | R   | Is the clearcore executing it's program                                                                                                              |
| 7     | is_ready_for_manual_control     | R   | Is the clearcore ready to take a commanded position. this is safety checks as well as making sure there is no cycle in progress                      |
| 20    | is_commanded_pos                | R/W | Has the HMI requested a new commanded position? If true it should read the commanded pos and go there, else the commanded pos is ignored             |
| 30    | is_rth_button_latched           | R/W | Has the 'return to home' button been pressed since last check? Gets set back to 0 on completion                                                      |
| 31    | is_axis_homing_button_latched   | R/W | Has the 'run axis homing sequence' button been pressed since last check? Gets set back to 0 on completion                                            |
| 32    | is_set_job_start_button_latched | R/W | Has the 'Set start to current position' button been pressed since last check? Gets set back to 0 on completion                                       |
| 33    | is_set_job_end_button_latched   | R/W | Has the 'Set end to current position' button been pressed since last check? Gets set back to 0 on completion                                         |
| 34    | is_set_job_park_button_latched  | R/W | Has the 'Set park to current position' button been pressed since last check? Gets set back to 0 on completion                                        |


# Hregs

| Index | Var name                 | R/W (from HMI) | Description                                                                                                                                                    |
|-------|--------------------------|----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2     | actual_position_upper    | R              | This is the >1 portion of the actual position of the axis in inches.                                                                                           |
| 3     | actual_position_lower    | R              | This is the <1 portion of the actual position of the axis in inches. Formula for combination is `actual_position_upper+(actual_position_lower/65536)`          |
| 12    | commanded_position_upper | R/W            | This is the >1 portion of the commanded position of the axis in inches. The commanded pos will not be acted upon if `is_commanded_pos` is not set              |
| 13    | commanded_position_lower | R/W            | This is the <1 portion of the commanded position of the axis in inches. Formula for combination is `commanded_position_upper+(commanded_position_lower/65536)` |
| 16    | job_progress             | R              | The progress of the current job if there is one. (job_progress/65536)*100%                                                                                     |
| 19    | job_start_pos_upper      | R              | This is the >1 portion of the job_start_pos of the axis in inches. (While the register is read only, the value can semantically changed otherwise)             |
| 20    | job_start_pos_lower      | R              | This is the <1 portion of the job_start_pos of the axis in inches. Formula for combination is `job_start_pos_upper+(job_start_pos_lower/65536)`                |
| 21    | job_end_pos_upper        | R              | Same as job_start_pos_upper but for end                                                                                                                        |
| 22    | job_end_pos_lower        | R              | Same as job_start_pos_lower but for end                                                                                                                        |
| 23    | job_park_pos_upper       | R              | Same as job_start_pos_upper but for park                                                                                                                       |
| 24    | job_park_pos_lower       | R              | Same as job_start_pos_lower but for park                                                                                                                       |
| 26    | max_pos_upper            | R              | This is the >1 portion of the maximum absolute position of the axis in inches.                                                                                 |
| 27    | max_pos_lower            | R              | This is the <1 portion of the maximum absolute position of the axis in inches. Formula for combination is `max_pos_upper+(max_pos_lower/65536)`                |
