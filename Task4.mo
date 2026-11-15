model FurutaPendulum "Furuta pendulum"
    
    parameter Real pendulumA_start = -Modelica.Constants.pi;
    
    model ControllerLQR
      parameter Real K[6] = {0.1, 0.03018962, 0.10349668, -0.00030896, 0.1051762, -0.00284298};
      Modelica.Blocks.Interfaces.RealInput phi, dphi, theta1, dtheta1, theta2, dtheta2;
      Modelica.Blocks.Interfaces.RealOutput u(start=0);
      Real x[6];
    equation
      x = {phi + 3.14/2, dphi, theta1 + 3.14, dtheta1, -(theta2 + 3.14), -dtheta2};
      u = -(K[1]*x[1] + K[2]*x[2] + K[3]*x[3] + K[4]*x[4] + K[5]*x[5] + K[6]*x[6]);
    end ControllerLQR;
    
    inner Modelica.Mechanics.MultiBody.World world(
      axisLength(displayUnit="mm"),
      axisDiameter(displayUnit="mm"),
      nominalLength(displayUnit="mm") = 0.1)
                                annotation (Placement(transformation(extent={{-60,-60},
              {-40,-40}},   rotation=0)));
    Modelica.Mechanics.MultiBody.Joints.Revolute rotor(
      a(fixed=false),
      w(fixed=true),
      cylinderLength(displayUnit="mm") = 0.015,
      cylinderColor={0,0,0},
      cylinderDiameter(displayUnit="mm") = 0.0605,
      useAxisFlange=true,
      n={0,1,0},
      phi(fixed=true, start=-1.5707963267949)) annotation (Placement(
          transformation(
          origin={0,32},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Joints.Revolute pendulumAxis(
      a(fixed=false),
      w(fixed=true),
      cylinderLength(displayUnit="mm") = 0.005,
      cylinderDiameter(displayUnit="mm") = 0.005,
      cylinderColor={200,200,200},
      useAxisFlange=true,
      n={-1,0,0},
      phi(
        fixed=true,
        start=pendulumA_start,
        displayUnit="rad")) annotation (Placement(transformation(extent={{38.0,56.0},{58.0,76.0}}, rotation=0.0,origin = {0.0,0.0})));
    Modelica.Mechanics.MultiBody.Parts.BodyCylinder base(
      r(displayUnit="mm") = {0,0.1,0},
      r_shape(displayUnit="mm") = {0,0,0},
      diameter(displayUnit="mm") = 0.06,
      color={155,155,155},
      r_0(displayUnit="mm", fixed=true)) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,-26})));
    Modelica.Mechanics.MultiBody.Parts.BodyCylinder pendulumArm(
      r_shape(displayUnit="mm") = {0,0,0},
      diameter(displayUnit="mm") = 0.005,
      color={200,200,200},
      r(displayUnit="mm") = {0,0.075,0},
      density=3700) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={70,2})));
    Modelica.Mechanics.MultiBody.Parts.Fixed fixed(
      length=0,
      width=0,
      height=0,
      r(displayUnit="mm") = {0,-0.025,-0.1})
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,-52})));
    Modelica.Mechanics.Rotational.Components.Damper pendulumDamper(d=0.000004)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={48,86})));
    Modelica.Mechanics.Rotational.Components.Damper rotorDamper(d=0.05) annotation (
        Placement(transformation(
          extent={{-10.0,-10.0},{10.0,10.0}},
          rotation=180.0,
          origin={-24.0,42.0})));
    Modelica.Mechanics.MultiBody.Parts.BodyCylinder pendulumAttachment(
      r_shape(displayUnit="mm") = {0,0,0},
      diameter(displayUnit="mm") = 0.005,
      color={155,155,155},
      r(displayUnit="mm") = {0.043,0,0},
      density=3700) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={14,66})));

    Modelica.Mechanics.Rotational.Sensors.AngleSensor pendulumA
      annotation (Placement(transformation(extent={{78,64},{92,78}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor pendulumW
      annotation (Placement(transformation(extent={{78,80},{92,94}})));
    Modelica.Mechanics.Rotational.Sensors.AngleSensor rotorA
      annotation (Placement(transformation(extent={{-20,-12},{-6,2}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor rotorW
      annotation (Placement(transformation(extent={{-20,2},{-6,16}})));
    Modelica.Blocks.Sources.Pulse pulse[3](
      startTime={1,1,1},
      width={1,1,1},
      period={10,10,10},
      amplitude={0.001,0,0})   annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={32,-32})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce disturbance
      annotation (Placement(transformation(extent={{50,-40},{66,-24}})));
    .Modelica.Mechanics.MultiBody.Parts.BodyCylinder pendulumAttachment2(density = 3700,r(displayUnit = "mm") = {0.043,0,0},color = {155,155,155},diameter(displayUnit = "mm") = 0.005,r_shape(displayUnit = "mm") = {0,0,0}) annotation(Placement(transformation(extent = {{-16.0,64.0},{-36.0,84.0}},rotation = 0.0,origin = {0.0,0.0})));
    .Modelica.Mechanics.MultiBody.Joints.Revolute pendulumAxis2(phi(fixed = true,start = pendulumA_start,displayUnit = "rad"),n = {1,0,0},useAxisFlange = true,cylinderColor = {200,200,200},cylinderDiameter(displayUnit = "mm") = 0.005,cylinderLength(displayUnit = "mm") = 0.005,w(fixed = true),a(fixed = false)) annotation(Placement(transformation(extent = {{-52.0,64.0},{-72.0,84.0}},rotation = 0.0,origin = {0.0,0.0})));
    .Modelica.Mechanics.Rotational.Components.Damper pendulumDamper2(d = 2.0E-6) annotation(Placement(transformation(extent = {{10.0,-10.0},{-10.0,10.0}},rotation = -180.0,origin = {-62.0,100.0})));
    .Modelica.Mechanics.Rotational.Sensors.AngleSensor pendulumA2 annotation(Placement(transformation(extent = {{-91.0,59.0},{-105.0,73.0}},rotation = 0.0,origin = {0.0,0.0})));
    .Modelica.Mechanics.Rotational.Sensors.SpeedSensor pendulumW2 annotation(Placement(transformation(extent = {{-91.0,81.0},{-105.0,95.0}},rotation = 0.0,origin = {0.0,0.0})));
    .Modelica.Mechanics.MultiBody.Parts.BodyCylinder pendulumArm2(density = 3700,r(displayUnit = "mm") = {0,0.030,0},color = {200,200,200},diameter(displayUnit = "mm") = 0.005,r_shape(displayUnit = "mm") = {0,0,0}) annotation(Placement(transformation(extent = {{-10.0,-10.0},{10.0,10.0}},rotation = -90.0,origin = {-82.0,24.0})));
    .Modelica.Mechanics.MultiBody.Forces.WorldForce disturbance2 annotation(Placement(transformation(extent = {{-105.0,-18.0},{-89.0,-2.0}},rotation = 0.0,origin = {0.0,0.0})));
    .Modelica.Blocks.Sources.Pulse pulse2[3](amplitude = {0.001,0,0},period = {10,10,10},width = {1,1,1},startTime = {1,1,1}) annotation(Placement(transformation(extent = {{-128.0,-16.0},{-116.0,-4.0}},rotation = 0.0,origin = {0.0,0.0})));
    .Modelica.Mechanics.Rotational.Sources.Torque torque annotation(Placement(transformation(extent = {{-64.0,-12.0},{-44.0,8.0}},origin = {0.0,0.0},rotation = 0.0)));
    
    ControllerLQR controller;
    equation
    connect(pendulumAxis.frame_b, pendulumArm.frame_a) annotation (Line(
        points={{58,66},{70,66},{70,12}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(base.frame_b, rotor.frame_a) annotation (Line(
        points={{0,-16},{0,0},{-6.66134e-16,0},{-6.66134e-16,22}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(base.frame_a, fixed.frame_b) annotation (Line(
        points={{0,-36},{0,-42},{4.44089e-16,-42}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pendulumDamper.flange_b, pendulumAxis.support) annotation (Line(
        points={{38,86},{30,86},{30,76},{42,76}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(pendulumDamper.flange_a, pendulumAxis.axis) annotation (Line(
        points={{58,86},{66,86},{66,76},{48,76}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(rotorDamper.flange_a, rotor.axis) annotation (Line(
        points={{-14,42},{-10,42},{-10,32}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(rotorDamper.flange_b, rotor.support) annotation (Line(
        points={{-34,42},{-40,42},{-40,26},{-10,26}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(pendulumAttachment.frame_b, pendulumAxis.frame_a) annotation (Line(
        points={{24,66},{38,66}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pendulumAttachment.frame_a, rotor.frame_b) annotation (Line(
        points={{4,66},{0,66},{0,42},{4.44089e-16,42}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pendulumW.flange, pendulumAxis.axis) annotation (Line(
        points={{78,87},{74,87},{74,76},{48,76}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(pendulumA.flange, pendulumAxis.axis) annotation (Line(
        points={{78,71},{74,71},{74,76},{48,76}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(rotorW.flange, rotor.axis) annotation (Line(
        points={{-20,9},{-24,9},{-24,32},{-10,32}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(rotorA.flange, rotor.axis) annotation (Line(
        points={{-20,-5},{-24,-5},{-24,32},{-10,32}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(pulse.y,disturbance. force) annotation (Line(
        points={{38.6,-32},{48.4,-32}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(disturbance.frame_b, pendulumArm.frame_b) annotation (Line(
        points={{66,-32},{70,-32},{70,-8}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pendulumAttachment2.frame_a,rotor.frame_b) annotation(Line(points = {{-16,74},{-4,74},{-4,48},{2.220446049250313e-15,48},{2.220446049250313e-15,42}},color = {95,95,95}));
    connect(pendulumAxis2.frame_a,pendulumAttachment2.frame_b) annotation(Line(points = {{-52,74},{-36,74}},color = {95,95,95}));
    connect(pulse2.y,disturbance2.force) annotation(Line(points = {{-115.4,-10},{-106.6,-10}},color = {0,0,127}));
    connect(pendulumArm2.frame_b,disturbance2.frame_b) annotation(Line(points = {{-82,14},{-82,-10},{-89,-10}},color = {95,95,95}));
    connect(pendulumAxis2.frame_b,pendulumArm2.frame_a) annotation(Line(points = {{-72,74},{-82,74},{-82,34}},color = {95,95,95}));
    connect(pendulumAxis2.support,pendulumDamper2.flange_b) annotation(Line(points = {{-56,84},{-56,90},{-46,90},{-46,100},{-52,100}},color = {0,0,0}));
    connect(pendulumDamper2.flange_a,pendulumAxis2.axis) annotation(Line(points = {{-72,100},{-78,100},{-78,90},{-62,90},{-62,84}},color = {0,0,0}));
    connect(pendulumAxis2.axis,pendulumW2.flange) annotation(Line(points = {{-62,84},{-62,90},{-85,90},{-85,88},{-91,88}},color = {0,0,0}));
    connect(pendulumAxis2.axis,pendulumA2.flange) annotation(Line(points = {{-62,84},{-62,90},{-85,90},{-85,66},{-91,66}},color = {0,0,0}));
    connect(torque.flange,rotor.axis) annotation(Line(points = {{-44,-2},{-27,-2},{-27,32},{-10,32}},color = {0,0,0}));
    connect(rotorA.phi,    controller.phi);
    connect(rotorW.w,      controller.dphi);
    connect(pendulumA.phi, controller.theta1);
    connect(pendulumW.w,   controller.dtheta1);
    connect(pendulumA2.phi, controller.theta2);
    connect(pendulumW2.w,  controller.dtheta2);
    connect(controller.u,  torque.tau);
    annotation (
      versionDate="2014-02-04",
      Commands(file="Furuta.mos" "Simulate Furuta pendulum", file="Animate.mos"
          "Animate Furuta pendulum"),
      experiment(NumberOfIntervals=5000, StopTime=10),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}),     graphics),uses(Modelica(version = "4.0.0")));
end FurutaPendulum;

