# PID Controller | Term 2 P4

This project required me to code and tune a PID controller to successfully drive the simulated car around the track. I accomplished this by using two PID controllers: one controlling the steering angle and the second controlling the throttle.

### Component Selection
The P, I and D components for the PID algorithm were selected using a mix of manual tuning and a Twiddle algorithm. I chose this hybrid approach in order to give the Twiddle algorithm a 'headstart' by seeding it with starting values that were reasonably effective. Additionally, with two PID controllers to be tuned, I suspected starting the Twiddle off in a blind search would take a very long time.

My PID algorithms outputted the corrections as follows:
#####Steering PID
`steer_value = pid_steering(cte)`

#####Throttle PID
`throttle_value = 1.0 - 2.0 * pid_throttle(std::abs(cte))`

The `cte` value is the error value based on how far off the vehicle veers from the center of the track. For the `steer_value` we use the actual value of `cte` (the sign tells us which direction the vehicle strays). For the throttle, we only use the magnitude; if we magnitude of the error is increasing, we'd like the PID response to increase and decrease throttle (and possibly brake).

The general PID algorithm is:

`PID response = -Kp*cte - Ki * sum(past_cte_values) - Kd * (current_cte - previous_cte)`

##### P Selection
The `P` component provides the weight to counteract any instantaneous error experienced at the current time step. Choosing a large value would result in over-/under-shooting behavior. By watching the car on a straightaway and on a turn, you can easily deduce a good starting value for this component. For the steering correction, my starting value was `0.10`. For the throttle PID my starting value was `0.05`.

##### I Selection
The `I` component will give weight to the sum of past errors. I used a `vector` of length `50` to store past `cte` error values. The sum of these is then multiplied by the `Ki` factor, giving the final PID result. For both steering and throttle I started with a value of `1e-3`. I arrived at this value manually by observing how much drift accrued from the center line when the vehicle was making hard turns.

##### D Selection
The `D` component controls the rate of change of the error. That is, if the error is changing fast, the error will be higher. For both the steering and throttle, I started with a value of `3.0`. Here also, I observed how the vehicle reacted to a turn and if the reaction was weak, I increased this until it looked reasonable.

#### Twiddle 
I set the `dp` values for each of the P-I-D components in relation to the seeded values; i.e., something close to the order of magnitude of the respective component. The Twiddle algorithm took a lot of time to converge to decent values and seeding good starting values greatly made this process efficient.

My resulting PID values were:

`p_steering = {0.0876, 1.58e-03, 3.808}`

`p_throttle = {0.05, 2e-3, 3.25}`

This setup resulted in several successful roundtrips around the track with a mean error of approximately `0.60` and with mean speed per lap between `60 - 65` mph.



