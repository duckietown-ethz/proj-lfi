Different nodes publish their own car_cmd. Inside the car_interface only
one gets mapped to /.../car_cmd_switch_node/cmd
The mapping is done by the car_cmd_switch_node, by reading the FSM Mode directly
This is done according to the following configurtation file
https://github.com/duckietown/dt-car-interface/blob/daffy/packages/dagu_car/config/car_cmd_switch_node/default.yaml
