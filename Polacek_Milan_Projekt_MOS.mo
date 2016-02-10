package Tradiator
  model pokojikSKotlem
    package Medium = Buildings.Media.Water "Medium model";
    parameter Modelica.SIunits.Temperature TRoo = 20 + 273.15 "Room temperature" annotation(Evaluate = false);
    parameter Modelica.SIunits.Power Q_flow_nominal = 500 "Nominal power";
    parameter Modelica.SIunits.Temperature T_a_nominal = 273.15 + 40 "Radiator inlet temperature at nominal condition" annotation(Evaluate = false);
    parameter Modelica.SIunits.Temperature T_b_nominal = 273.15 + 30 "Radiator outlet temperature at nominal condition" annotation(Evaluate = false);
    parameter Modelica.SIunits.MassFlowRate m_flow_nominal = Q_flow_nominal / (T_a_nominal - T_b_nominal) / Medium.cp_const "Nominal mass flow rate";
    parameter Modelica.SIunits.Pressure dp_nominal = 3000 "Pressure drop at m_flow_nominal";
    Buildings.Fluid.Actuators.Valves.TwoWayLinear val annotation(Placement(visible = true, transformation(origin = {-30, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 rad1(redeclare package Medium = Medium, T_a_nominal = T_a_nominal, T_b_nominal = T_b_nominal, Q_flow_nominal = Q_flow_nominal, TAir_nominal = TRoo, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator" annotation(Placement(visible = true, transformation(extent = {{36, -2}, {56, 18}}, rotation = 0)));
    Buildings.Fluid.FixedResistances.FixedResistanceDpM res annotation(Placement(visible = true, transformation(origin = {6, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.FixedResistances.FixedResistanceDpM res1 annotation(Placement(visible = true, transformation(origin = {74, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.Delays.DelayFirstOrder del annotation(Placement(visible = true, transformation(origin = {36, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Buildings.Fluid.Storage.ExpansionVessel exp annotation(Placement(visible = true, transformation(origin = {36, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTem annotation(Placement(visible = true, transformation(origin = {16, 42}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Buildings.Controls.Continuous.LimPID conPID annotation(Placement(visible = true, transformation(origin = {-30, 42}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
    Buildings.Fluid.Movers.FlowControlled_dp pump(m_flow_nominal = m_flow_nominal, dp(start = dp_nominal)) annotation(Placement(visible = true, transformation(origin = {-62, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.FixedResistances.FixedResistanceDpM res2 annotation(Placement(visible = true, transformation(origin = {-38, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Buildings.Fluid.HeatExchangers.HeaterCooler_u hea annotation(Placement(visible = true, transformation(origin = {-4, -62}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant dp(k = 3600) annotation(Placement(visible = true, transformation(origin = {-76, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant T(k = 3600) annotation(Placement(visible = true, transformation(origin = {-16, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant TSet(k = 3600) annotation(Placement(visible = true, transformation(origin = {-46, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(dp.y, pump.dp_in) annotation(Line(points = {{-65, 46}, {-62, 46}, {-62, 20}, {-62, 20}}, color = {0, 0, 127}));
    connect(TSet.y, conPID.u_s) annotation(Line(points = {{-35, 82}, {-30, 82}, {-30, 54}}, color = {0, 0, 127}));
    connect(T.y, hea.u) annotation(Line(points = {{-5, -32}, {16, -32}, {16, -56}, {8, -56}, {8, -56}}, color = {0, 0, 127}));
    connect(hea.port_b, res2.port_a) annotation(Line(points = {{-14, -62}, {-28, -62}, {-28, -62}, {-28, -62}}, color = {0, 127, 255}));
    connect(del.ports[3], hea.port_a) annotation(Line(points = {{36, -62}, {6, -62}, {6, -62}, {6, -62}}, color = {0, 127, 255}));
    connect(pump.port_a, res2.port_b) annotation(Line(points = {{-72, 8}, {-82, 8}, {-82, -62}, {-48, -62}}, color = {0, 127, 255}));
    connect(pump.port_b, val.port_a) annotation(Line(points = {{-52, 8}, {-40, 8}, {-40, 8}, {-40, 8}}, color = {0, 127, 255}));
    connect(senTem.port_b, conPID.u_m) annotation(Line(points = {{6, 42}, {-18, 42}}, color = {0, 127, 255}));
    connect(conPID.y, val.y) annotation(Line(points = {{-30, 31}, {-30, 20}}, color = {0, 0, 127}));
    connect(rad1.heatPortCon, senTem.port_a) annotation(Line(points = {{44, 15}, {42, 15}, {42, 42}, {26, 42}, {26, 42}}, color = {191, 0, 0}));
    connect(exp.port_a, del.ports[2]) annotation(Line(points = {{36, -46}, {36, -46}, {36, -62}, {36, -62}}, color = {0, 127, 255}));
    connect(res1.port_b, del.ports[1]) annotation(Line(points = {{84, 8}, {92, 8}, {92, -62}, {40, -62}, {40, -62}}, color = {0, 127, 255}));
    connect(rad1.port_b, res1.port_a) annotation(Line(points = {{56, 8}, {64, 8}, {64, 8}, {64, 8}}, color = {0, 127, 255}));
    connect(res.port_b, rad1.port_a) annotation(Line(points = {{16, 8}, {36, 8}, {36, 8}, {36, 8}, {36, 8}}, color = {0, 127, 255}));
    connect(val.port_b, res.port_a) annotation(Line(points = {{-20, 8}, {-4, 8}, {-4, 8}, {-4, 8}}, color = {0, 127, 255}));
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
  end pokojikSKotlem;

  model pokojSregulaci
    package Medium1 = Buildings.Media.Water;
    package Medium2 = Buildings.Media.Air;
    parameter Modelica.SIunits.Temperature T_a1_nominal = 5 + 273.15;
    parameter Modelica.SIunits.Temperature T_b1_nominal = 10 + 273.15;
    parameter Modelica.SIunits.Temperature T_a2_nominal = 30 + 273.15;
    parameter Modelica.SIunits.Temperature T_b2_nominal = 15 + 273.15;
    parameter Modelica.SIunits.MassFlowRate m1_flow_nominal = 0.1 "Nominal mass flow rate medium 1";
    parameter Modelica.SIunits.MassFlowRate m2_flow_nominal = m1_flow_nominal * 4200 / 1000 * (T_a1_nominal - T_b1_nominal) / (T_b2_nominal - T_a2_nominal) "Nominal mass flow rate medium 2";
    Buildings.Fluid.Sources.Boundary_pT sin_2(redeclare package Medium = Medium2, nPorts = 1, use_p_in = false, p(displayUnit = "Pa") = 101325, T = 303.15) annotation(Placement(visible = true, transformation(extent = {{-96, -6}, {-76, 14}}, rotation = 0)));
    Buildings.Fluid.Sources.Boundary_pT sou_2(redeclare package Medium = Medium2, nPorts = 1, T = T_a2_nominal, X = {0.02, 1 - 0.02}, use_T_in = true, use_X_in = true, p(displayUnit = "Pa") = 101325 + 300) annotation(Placement(visible = true, transformation(extent = {{124, -6}, {104, 14}}, rotation = 0)));
    Buildings.Fluid.Sources.Boundary_pT sin_1(redeclare package Medium = Medium1, nPorts = 1, use_p_in = false, p = 300000, T = 293.15) annotation(Placement(visible = true, transformation(extent = {{124, 34}, {104, 54}}, rotation = 0)));
    Buildings.Fluid.Sources.Boundary_pT sou_1(redeclare package Medium = Medium1, nPorts = 1, use_T_in = true, p = 300000 + 12000) annotation(Placement(visible = true, transformation(extent = {{-56, 34}, {-36, 54}}, rotation = 0)));
    Buildings.Fluid.Sensors.TemperatureTwoPort temSen(redeclare package Medium = Medium2, m_flow_nominal = m2_flow_nominal) annotation(Placement(visible = true, transformation(extent = {{4, -6}, {-16, 14}}, rotation = 0)));
    Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage val(redeclare package Medium = Medium1, m_flow_nominal = m1_flow_nominal, dpValve_nominal = 6000) "Valve model" annotation(Placement(visible = true, transformation(extent = {{14, 34}, {34, 54}}, rotation = 0)));
    Modelica.Blocks.Sources.TimeTable TSet(table = [0, 288.15; 600, 288.15; 600, 298.15; 1200, 298.15; 1800, 283.15; 2400, 283.15; 2400, 288.15]) "Setpoint temperature" annotation(Placement(visible = true, transformation(extent = {{-56, 74}, {-36, 94}}, rotation = 0)));
    Buildings.Fluid.HeatExchangers.WetCoilCounterFlow hex(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, m1_flow_nominal = m1_flow_nominal, m2_flow_nominal = m2_flow_nominal, dp2_nominal(displayUnit = "Pa") = 200, allowFlowReversal1 = true, allowFlowReversal2 = true, dp1_nominal(displayUnit = "Pa") = 3000, UA_nominal = 2 * m1_flow_nominal * 4200 * (T_a1_nominal - T_b1_nominal) / Buildings.Fluid.HeatExchangers.BaseClasses.lmtd(T_a1_nominal, T_b1_nominal, T_a2_nominal, T_b2_nominal), show_T = true, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial) annotation(Placement(visible = true, transformation(extent = {{44, 0}, {64, 20}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const(k = 0.8) annotation(Placement(visible = true, transformation(extent = {{84, -86}, {104, -66}}, rotation = 0)));
    Buildings.Utilities.Psychrometrics.X_pTphi x_pTphi(use_p_in = false) annotation(Placement(visible = true, transformation(extent = {{134, -58}, {154, -38}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const1(k = T_a2_nominal) annotation(Placement(visible = true, transformation(extent = {{84, -54}, {104, -34}}, rotation = 0)));
    Buildings.Controls.Continuous.LimPID con(Td = 1, reverseAction = true, yMin = 0, controllerType = Modelica.Blocks.Types.SimpleController.PI, k = 0.1, Ti = 60) "Controller" annotation(Placement(visible = true, transformation(extent = {{-16, 74}, {4, 94}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp TWat(height = 30, offset = T_a1_nominal, startTime = 300, duration = 2000) "Water temperature, raised to high value at t=3000 s" annotation(Placement(visible = true, transformation(extent = {{-96, 38}, {-76, 58}}, rotation = 0)));
    Buildings.Fluid.FixedResistances.FixedResistanceDpM res(from_dp = true, redeclare package Medium = Medium2, dp_nominal = 100, m_flow_nominal = m2_flow_nominal) annotation(Placement(visible = true, transformation(origin = {-44, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Buildings.Fluid.FixedResistances.FixedResistanceDpM res1(from_dp = true, redeclare package Medium = Medium1, dp_nominal = 3000, m_flow_nominal = m1_flow_nominal) annotation(Placement(visible = true, transformation(origin = {84, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(res1.port_a, hex.port_b1) annotation(Line(points = {{74, 44}, {68, 44}, {68, 16}, {64, 16}, {64, 16}}, color = {0, 127, 255}));
    connect(res1.port_b, sin_1.ports[1]) annotation(Line(points = {{94, 44}, {104, 44}, {104, 44}, {104, 44}}, color = {0, 127, 255}));
    connect(res.port_b, sin_2.ports[1]) annotation(Line(points = {{-54, 0}, {-66, 0}, {-66, 4}, {-76, 4}, {-76, 4}}, color = {0, 127, 255}));
    connect(temSen.port_b, res.port_b) annotation(Line(points = {{-16, 4}, {-24, 4}, {-24, 0}, {-54, 0}, {-54, 0}, {-54, 0}}, color = {0, 127, 255}));
    connect(TWat.y, sou_1.T_in) annotation(Line(points = {{-75, 48}, {-58, 48}}, color = {0, 0, 127}));
    connect(con.y, val.y) annotation(Line(points = {{5, 84}, {24, 84}, {24, 56}}, color = {0, 0, 127}));
    connect(temSen.T, con.u_m) annotation(Line(points = {{-6, 15}, {-6, 72}}, color = {0, 0, 127}));
    connect(TSet.y, con.u_s) annotation(Line(points = {{-35, 84}, {-18, 84}}, color = {0, 0, 127}));
    connect(const1.y, sou_2.T_in) annotation(Line(points = {{105, -44}, {110.5, -44}, {110.5, -46}, {118, -46}, {118, -18}, {144, -18}, {144, 6}, {134, 6}, {134, 8}, {126, 8}}, color = {0, 0, 127}));
    connect(const1.y, x_pTphi.T) annotation(Line(points = {{105, -44}, {118, -44}, {118, -50}, {125, -50}, {125, -48}, {132, -48}}, color = {0, 0, 127}));
    connect(const.y, x_pTphi.phi) annotation(Line(points = {{105, -76}, {120, -76}, {120, -56}, {126, -56}, {126, -54}, {132, -54}}, color = {0, 0, 127}));
    connect(x_pTphi.X, sou_2.X_in) annotation(Line(points = {{155, -48}, {162, -48}, {162, -52}, {170, -52}, {170, 0}, {126, 0}}, color = {0, 0, 127}));
    connect(hex.port_b2, temSen.port_a) annotation(Line(points = {{44, 4}, {4, 4}}, color = {0, 127, 255}));
    connect(sou_2.ports[1], hex.port_a2) annotation(Line(points = {{104, 4}, {64, 4}}, color = {0, 127, 255}));
    connect(val.port_b, hex.port_a1) annotation(Line(points = {{34, 44}, {34, 44}, {34, 42}, {36, 42}, {36, 14}, {39, 14}, {39, 16}, {44, 16}}, color = {0, 127, 255}));
    connect(sou_1.ports[1], val.port_a) annotation(Line(points = {{-36, 44}, {14, 44}}, color = {0, 127, 255}));
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
  end pokojSregulaci;

  model pokojModelica
    constant Real absZeros = 273.15;
    Real tempRoom = absZeros + 20 "20 degC in room";
    replaceable package Medium = Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium;
    Modelica.Fluid.Pipes.DynamicPipe radiator(length = 2, diameter = 1) annotation(Placement(visible = true, transformation(origin = {26, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Fluid.Pipes.DynamicPipe heater(length = 1, diameter = 0.1) annotation(Placement(visible = true, transformation(origin = {-46, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Fluid.Valves.ValveIncompressible valveIncompressible1 annotation(Placement(visible = true, transformation(origin = {-10, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step1(offset = 0.5, startTime = 10) annotation(Placement(visible = true, transformation(origin = {-34, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor room(G = 1.6e3 / 20) annotation(Placement(visible = true, transformation(origin = {26, 50}, extent = {{8, -10}, {-8, 10}}, rotation = 90)));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor1(G = 1.6e3 / 20) annotation(Placement(visible = true, transformation(origin = {-46, 50}, extent = {{8, -10}, {-8, 10}}, rotation = -90)));
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature T_ambient(T = tempRoom) annotation(Placement(visible = true, transformation(extent = {{-4, 59}, {10, 73}}, rotation = 0)));
    Modelica.Fluid.Vessels.OpenTank tank(height = 1, crossArea = 0.1) annotation(Placement(visible = true, transformation(origin = {-72, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature flame(T = absZeros + 40) annotation(Placement(visible = true, transformation(extent = {{-70, 71}, {-56, 85}}, rotation = 0)));
    Modelica.Fluid.Machines.Pump pump1(N_nominal = 1) annotation(Placement(visible = true, transformation(origin = {-34, -16}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(pump1.port_b, tank.ports[2]) annotation(Line(points = {{-44, -16}, {-74, -16}, {-74, 38}, {-74, 38}}, color = {0, 127, 255}));
    connect(radiator.port_b, pump1.port_a) annotation(Line(points = {{36, 28}, {52, 28}, {52, -16}, {-24, -16}, {-24, -16}}, color = {0, 127, 255}));
    connect(flame.port, thermalConductor1.port_b) annotation(Line(points = {{-56, 78}, {-46, 78}, {-46, 58}, {-46, 58}}, color = {191, 0, 0}));
    connect(tank.ports[1], heater.port_a) annotation(Line(points = {{-72, 38}, {-70, 38}, {-70, 28}, {-56, 28}, {-56, 28}}, color = {0, 127, 255}));
    connect(room.port_a, T_ambient.port) annotation(Line(points = {{26, 58}, {18, 58}, {18, 66}, {10, 66}}, color = {191, 0, 0}));
    connect(thermalConductor1.port_a, heater.heatPorts[1]) annotation(Line(points = {{-46, 42}, {-46, 42}, {-46, 32}, {-46, 32}}, color = {191, 0, 0}));
    connect(room.port_b, radiator.heatPorts[1]) annotation(Line(points = {{26, 42}, {26, 32}}, color = {191, 0, 0}));
    connect(step1.y, valveIncompressible1.opening) annotation(Line(points = {{-23, 78}, {-10, 78}, {-10, 36}}, color = {0, 0, 127}));
    connect(valveIncompressible1.port_b, radiator.port_a) annotation(Line(points = {{0, 28}, {16, 28}, {16, 28}, {16, 28}}, color = {0, 127, 255}));
    connect(heater.port_b, valveIncompressible1.port_a) annotation(Line(points = {{-36, 28}, {-20, 28}, {-20, 28}, {-20, 28}}, color = {0, 127, 255}));
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
  end pokojModelica;

  model basicModelHTransf
    parameter Modelica.SIunits.Temperature T_final_K(fixed = false) "Projected final temperature";
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor mass1(C = 15, T(start = 373.15, fixed = true)) annotation(Placement(transformation(extent = {{-100, 20}, {-40, 80}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor mass2(C = 15, T(start = 273.15, fixed = true)) annotation(Placement(transformation(extent = {{40, 20}, {100, 80}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor conduction(G = 10) annotation(Placement(transformation(extent = {{-30, -20}, {30, 40}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor Tsensor1 annotation(Placement(transformation(extent = {{-60, -80}, {-20, -40}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor Tsensor2 annotation(Placement(transformation(extent = {{60, -80}, {20, -40}}, rotation = 0)));
  equation
    connect(mass1.port, conduction.port_a) annotation(Line(points = {{-70, 20}, {-70, 10}, {-30, 10}}, color = {191, 0, 0}));
    connect(conduction.port_b, mass2.port) annotation(Line(points = {{30, 10}, {70, 10}, {70, 20}}, color = {191, 0, 0}));
    connect(mass1.port, Tsensor1.port) annotation(Line(points = {{-70, 20}, {-70, -60}, {-60, -60}}, color = {191, 0, 0}));
    connect(mass2.port, Tsensor2.port) annotation(Line(points = {{70, 20}, {70, -60}, {60, -60}}, color = {191, 0, 0}));
  initial equation
    T_final_K = (mass1.T * mass1.C + mass2.T * mass2.C) / (mass1.C + mass2.C);
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
  end basicModelHTransf;
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
end Tradiator;