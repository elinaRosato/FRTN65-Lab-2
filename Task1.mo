model FurutaDAE "Task 1: Basic Equation-based Furuta pendulum model"

  // Constants
  parameter Real alpha = 135e-6;
  parameter Real beta  = 10.2e-6;
  parameter Real gamma_c = 8.79e-6;
  parameter Real delta = 2.01e-3;
  parameter Real d_phi   = 0.05;
  parameter Real d_theta = 0.000004;

  // State variables
  Real phi(start = -Modelica.Constants.pi/2, fixed = true);
  Real dphi(start = 0, fixed = true);
  Real theta(start = -0.1, fixed = true); //upright position
  Real dtheta(start = 0, fixed = true);

  // Torques
  Real tau_phi;
  Real tau_theta;

equation
  // No input torque (u = 0 for parameter tuning)
  tau_phi   =  -d_phi * dphi;
  tau_theta = -d_theta * dtheta;

  // Kinematic equations
  der(phi)   = dphi;
  der(theta) = dtheta;

  // Equations of motion
  (alpha + beta*sin(theta)^2)*der(dphi)
    + gamma_c*cos(theta)*der(dtheta)
    + 2*beta*cos(theta)*sin(theta)*dphi*dtheta
    - gamma_c*sin(theta)*dtheta^2
    = tau_phi;

  gamma_c*cos(theta)*der(dphi)
    + beta*der(dtheta)
    - beta*cos(theta)*sin(theta)*dphi^2
    - delta*sin(theta)
    = tau_theta;

end FurutaDAE;
