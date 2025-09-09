# General introduction for package designed for flamingo, a aerial-aquatic bi-rotor robot
## background
## structure
##
### urdf
```
 #### rotor joint (virtual) for aerial rotor ####
    <joint name="rotor${self}" type="continuous">
      <limit effort="100.0" lower="${air_min_force}" upper="${air_max_force}" velocity="0.5"/>
      <parent link="rotor_parent${self}"/>
      <child link="thrust${self}"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <axis xyz="0 0 ${rotor_direction}"/>
    </joint>

    #### rotor joint (virtual) for aquatic rotor####
    <joint name="rotor${self + 2}" type="continuous">
      <limit effort="100.0" lower="${water_min_force}" upper="${water_max_force}" velocity="0.5"/>
      <parent link="rotor_parent${self}"/>
      <child link="thrust${self + 2}"/>
      <!-- modify this position -->
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <axis xyz="0 0 ${rotor_direction}"/>
    </joint>

    #### hardware interface ####
    <transmission name="rotor_tran${self}">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="rotor${self}">
        <hardwareInterface>RotorInterface</hardwareInterface>
      </joint>
      <actuator name="rotor_actuator${self}">
        <hardwareInterface>RotorInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>
```
- Link "Thrust" will the application point of actuator force and relevant torque will be applied at the point of "rotor_parent"
## todo
- [ ]  rviz configuration for coxial bi-rotor
- [ ]  state judgement node
- [ ]  motor's index should be classified into two groups
  - [ ]  can use index for each motor to enable general adaption
- [ ]  aerial controller
- [ ]  aquatic controller
- [ ]  overall pipeline to enable quick adapation for different kinds of controller for different domain
  - [ ]  should also accomodate different motors
- [ ]  add underwater dynamics
