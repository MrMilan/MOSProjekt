within ;
package Tradiator
  model pokojikSKotlem
    package MediumW = Buildings.Media.Water "Medium model";
    parameter Modelica.SIunits.Temperature TRoo = 20 + 273.15
      "Room temperature"                                                         annotation(Evaluate = false);
    parameter Modelica.SIunits.Power Q_flow_nominal = 1000 "Nominal power";
    parameter Modelica.SIunits.Temperature T_a_nominal = 273.15 + 40
      "Radiator inlet temperature at nominal condition"                                                                annotation(Evaluate = false);
    parameter Modelica.SIunits.Temperature T_b_nominal = 273.15 + 30
      "Radiator outlet temperature at nominal condition"                                                                annotation(Evaluate = false);
    parameter Modelica.SIunits.MassFlowRate m_flow_nominal = Q_flow_nominal / (T_a_nominal - T_b_nominal) / MediumW.cp_const
      "Nominal mass flow rate";
    parameter Modelica.SIunits.Pressure dp_nominal = 300
      "Pressure drop at m_flow_nominal";
    Buildings.Fluid.Actuators.Valves.TwoWayLinear val(
      redeclare package Medium = MediumW,
      m_flow_nominal=m_flow_nominal,
      dpValve_nominal=10)                             annotation(Placement(visible = true, transformation(origin = {-30, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

    Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 rad1(
                                  T_a_nominal = T_a_nominal, T_b_nominal = T_b_nominal, Q_flow_nominal = Q_flow_nominal, TAir_nominal = TRoo, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial,
      redeclare package Medium = MediumW) "Radiator"                        annotation(Placement(visible = true, transformation(extent = {{36, -2}, {56, 18}}, rotation = 0)));
    Buildings.Fluid.FixedResistances.FixedResistanceDpM res(
      redeclare package Medium = MediumW,
      m_flow_nominal=m_flow_nominal,
      dp_nominal=dp_nominal)                                        annotation(Placement(visible = true, transformation(origin = {6, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

    Buildings.Fluid.FixedResistances.FixedResistanceDpM res1(m_flow_nominal=m_flow_nominal,
        dp_nominal=dp_nominal,
      redeclare package Medium = MediumW)                            annotation(Placement(visible = true, transformation(origin = {74, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.Delays.DelayFirstOrder del(redeclare package Medium = MediumW, m_flow_nominal=m_flow_nominal,
      nPorts=3)                                annotation(Placement(visible = true, transformation(origin={36,-72},    extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Buildings.Fluid.Storage.ExpansionVessel exp(
      redeclare package Medium = MediumW,
      V_start=1,
      p=100000)                                 annotation(Placement(visible = true, transformation(origin={70,-40},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));

    Buildings.Fluid.Movers.FlowControlled_dp pump(m_flow_nominal = m_flow_nominal, dp(start = dp_nominal),
      redeclare package Medium = MediumW,
      m_flow(start=m_flow_nominal))                                                                                    annotation(Placement(visible = true, transformation(origin = {-62, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

    Buildings.Fluid.FixedResistances.FixedResistanceDpM res2(
      redeclare package Medium = MediumW,
      m_flow_nominal=m_flow_nominal,
      dp_nominal=dp_nominal)                                         annotation(Placement(visible = true, transformation(origin={-62,-62},    extent = {{-10, -10}, {10, 10}}, rotation = 180)));

    Modelica.Blocks.Sources.Constant dp(k=1)      annotation(Placement(visible = true, transformation(origin = {-76, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant VentOpen(k=1) annotation (Placement(visible=true, transformation(
          origin={-46,82},
          extent={{-10,-10},{10,10}},
          rotation=0)));
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature T_ambient(T = TRoo) annotation(Placement(visible = true, transformation(extent={{-2,41},
              {12,55}},                                                                                                    rotation = 0)));
    Buildings.Fluid.Sensors.TemperatureTwoPort temSen(redeclare package Medium
        =        MediumW, m_flow_nominal=m_flow_nominal,
      T_start=293.15)
      annotation (Placement(transformation(extent={{14,-72},{-6,-52}})));
    Modelica.Blocks.Sources.Pulse SwiHea(
      offset=273.15 + 30,
      amplitude=40,
      width=30,
      period=20,
      nperiod=3) "Setpoint temperature"             annotation (Placement(transformation(extent={{34,-28},
              {14,-8}})));
    Buildings.Fluid.HeatExchangers.HeaterCooler_T hea(
      redeclare package Medium = MediumW,
      m_flow_nominal=m_flow_nominal,
      dp_nominal=1000,
      Q_flow_maxCool=0,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      Q_flow_maxHeat=Q_flow_nominal) "Heater"
      annotation (Placement(transformation(extent={{-20,-72},{-40,-52}})));
  equation
    connect(dp.y, pump.dp_in) annotation(Line(points={{-65,46},{-62,46},{-62,20},{
            -62.2,20}},                                                                              color = {0, 0, 127}));
    connect(pump.port_a, res2.port_b) annotation(Line(points={{-72,8},{-82,8},{-82,-62},{-72,-62}},          color = {0, 127, 255}));
    connect(pump.port_b, val.port_a) annotation(Line(points={{-52,8},{-40,8}},                          color = {0, 127, 255}));
    connect(rad1.port_b, res1.port_a) annotation(Line(points={{56,8},{64,8}},                        color = {0, 127, 255}));
    connect(res.port_b, rad1.port_a) annotation(Line(points={{16,8},{36,8}},                                 color = {0, 127, 255}));
    connect(val.port_b, res.port_a) annotation(Line(points={{-20,8},{-4,8}},                        color = {0, 127, 255}));
    connect(VentOpen.y, val.y) annotation (Line(points={{-35,82},{-30,82},{-30,52},{-30,20}}, color={0,0,127}));
    connect(exp.port_a, del.ports[1]) annotation (Line(points={{70,-50},{36,-50},
            {36,-62},{38.6667,-62}},                                               color={0,127,255}));
    connect(res1.port_b, del.ports[2])
      annotation (Line(points={{84,8},{94,8},{94,6},{94,-62},{36,-62}},      color={0,127,255}));
    connect(T_ambient.port, rad1.heatPortCon)
      annotation (Line(points={{12,48},{12,48},{44,48},{44,15.2}}, color={191,0,0}));
    connect(temSen.port_a, del.ports[3]) annotation (Line(points={{14,-62},{
            33.3333,-62}},                                                                 color={0,127,255}));
    connect(res2.port_a, hea.port_b) annotation (Line(points={{-52,-62},{-40,-62}}, color={0,127,255}));
    connect(temSen.port_b, hea.port_a) annotation (Line(points={{-6,-62},{-14,-62},{-20,-62}}, color={0,127,255}));
    connect(SwiHea.y, hea.TSet)
      annotation (Line(points={{13,-18},{10,-18},{10,-36},{-18,-36},{-18,-56}}, color={0,0,127}));
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent={{-100,
              -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2, 2})));
  end pokojikSKotlem;

  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})),
    uses(Modelica(version="3.2.1"), Buildings(version="2.1.0")));
end Tradiator;
