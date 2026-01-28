from rocketpy import Environment, Flight, Rocket, SolidMotor


huntsville = Environment(latitude = 34.738228, longitude = -86.601791, elevation = 195)
huntsville.set_date((2026, 2,6, 12))
huntsville.set_atmospheric_model(type="Forecast", file="GFS")
#huntsville.info()

thrust_source = [
    (0.00, 0),
    (0.02, 800),
    (0.05, 1350),
    (0.10, 1500),
    (0.20, 1450),
    (0.40, 1380),
    (0.60, 1320),
    (0.80, 1280),
    (1.00, 1260),
    (1.20, 1250),
    (1.50, 1240),
    (1.80, 1230),
    (2.10, 1200),
    (2.40, 1100),
    (2.70, 700),
    (2.90, 300),
    (3.00, 0),
]

motor = SolidMotor(
    # Thrust curve (use thrustcurve.org data converted to .eng)
    thrust_source=thrust_source,

    # Mass properties
    dry_mass=1.93,  # kg (RMS-75/3840 hardware + nozzle + closures)
    dry_inertia=(0.145, 0.145, 0.0045),  # kg·m² (estimated solid cylinder)

    # Nozzle
    nozzle_radius=0.0375,  # m (75 mm / 2)
    throat_radius=0.011,   # m (≈11 mm, matches L1256 throat data)
    nozzle_position=-0.781,   # nozzle is reference

    # Grain configuration
    grain_number=5,
    grain_density=1815,  # kg/m³ (White Lightning)
    grain_outer_radius=0.033,  # m
    grain_initial_inner_radius=0.015,  # m
    grain_initial_height=0.110,  # m (≈110 mm per grain)
    grain_separation=0.005,  # m

    # Geometry / mass distribution
    grains_center_of_mass_position=0.15,  # m from nozzle (estimated)
    center_of_dry_mass_position=0.15,     # m from nozzle

    # Burn characteristics
    burn_time=3.0,  # s

    # Coordinate convention
    coordinate_system_orientation="nozzle_to_combustion_chamber",
)

#motor.info()


cantaloupe = Rocket(
    radius=0.0655,
    mass=13.356,
    inertia=(8.89, 8.89, 0.035),
    power_off_drag=0.439,
    power_on_drag=0.616,
    center_of_mass_without_motor=0,
    coordinate_system_orientation="tail_to_nose",
)

rail_buttons = cantaloupe.set_rail_buttons(
    upper_button_position=-0.315,
    lower_button_position=-0.600,
    angular_position=45,
)

cantaloupe.add_motor(motor, position=-0.805)

nose_cone = cantaloupe.add_nose(length=0.686, kind="vonKarman", position=1.435)

fin_set = cantaloupe.add_trapezoidal_fins(
    n=4,
    root_chord=0.230,
    tip_chord=0.146,
    span=0.127,
    position=-1.215,
)

Main = cantaloupe.add_parachute(
    "Main",
    cd_s=2.7824,
    trigger=152.4,
)

Drogue = cantaloupe.add_parachute(
    "Drogue",
    cd_s=0.9186,
    trigger="apogee",
)




#cantaloupe.all_info()

test_flight = Flight(
    rocket=cantaloupe, environment=huntsville, rail_length=4, inclination=85, heading=0
)

test_flight.plots.all()