import matplotlib.pyplot as plt
from raysect.primitive import Box, Cylinder, Sphere, Subtract
from raysect.optical import World, Point3D, translate, rotate, Vector3D
from raysect.optical.material import Lambert, UniformSurfaceEmitter
from raysect.optical.observer import PinholeCamera
from raysect.optical.observer.pipeline import RGBPipeline2D, PowerPipeline0D
from raysect.optical.spectralfunction import ConstantSF
from raysect.optical.observer.nonimaging import Pixel
from raysect.optical.library import schott
from raysect.optical.material.absorber import AbsorbingSurface

import math

def create_scene():
    world = World()

    # Materials
    ceiling_material = Lambert(ConstantSF(0.9))  
    wall_material = Lambert(ConstantSF(0.9))        # Eggshell white for walls
    east_wall_material = Lambert(ConstantSF(0.4))   # Eggshell white for walls
    floor_material = Lambert(ConstantSF(0.7))       # Light tan color
    table_material = Lambert(ConstantSF(0.5))       # Gray color

    emission_intensity1 = 50.0  #LED emission spectrum and intensity
    led_material = UniformSurfaceEmitter(ConstantSF(emission_intensity1))  # Bright LED emitter

    emission_intensity2 = 50.0  # 2nd LED emission spectrum and intensity
    second_LED = ConstantSF(emission_intensity2)

  # LED light source on the ceiling
  #    Cylinder(0.1, 0.9, transform=translate(2, 2.85, 4.5), parent=world, material=led_material) #LED center of room
    Cylinder(0.1, 0.9, transform=translate(1, 2.85, 4.5), parent=world, material=led_material) #LED towards left wall
    Cylinder(0.1, 0.9, transform=translate(3, 2.85, 4.5), parent=world, material=led_material) #LED towards right wall

    # 2nd LED emitter 
    led_diameter = .15 #2.4e-3  # 2.4 mm in meters
    led_radius = led_diameter / 2
    led_thickness = 0.25  #1e-5  # Thin disk approximation
    led_position = translate(2, 2.9, 4.5)
    Cylinder(led_radius, led_thickness, transform=led_position * rotate(0, 0, 0), parent=world, material=UniformSurfaceEmitter(second_LED))

  # test window transparency with tube light
    Cylinder(.05, 2.0, transform=translate(5, 2, 5.5)  * rotate(90, 0, 0), parent=world, material=led_material)

# room features

  # Window corners
    glass_material = schott('N-BK7')  # Using N-BK7 glass from the Schott library
    window_lower = Point3D(4, 0.5, 4.0)
    window_upper = Point3D(4.2, 2.5, 5.0)
    window = Box(window_lower, window_upper, material=glass_material)
 
  # Add east wall with window
    east_wall = Box(Point3D(4, 0, 0), Point3D(4.1, 3, 6), material=wall_material)
    east_wall_with_window = Subtract(east_wall, window, parent=world, material=east_wall_material)

  # Add the ceiling
    Box(
    Point3D(0, 2.9, 0),  # Lower corner just below the ceiling height
    Point3D(4, 3, 6),    # Upper corner at the ceiling height
    parent=world, material=ceiling_material)

  # West Wall (x = 0)
    west_wall = Box(
    Point3D(0, 0, 0),        # Lower corner
    Point3D(0.1, 3, 6),      # Upper corner (0.1m thick wall)
    parent=world, material=wall_material)

  # South Wall (z = 0)
    south_wall = Box(
    Point3D(0, 0, 0),        # Lower corner
    Point3D(4, 3, 0.1),      # Upper corner (0.1m thick wall)
    parent=world, material=wall_material)

  # North Wall (z = 6)
    north_wall = Box(
    Point3D(0, 0, 5.9),      # Lower corner
    Point3D(4, 3, 6),        # Upper corner (0.1m thick wall)
    parent=world, material=wall_material)

  #  Box(Point3D(0, 0, 0), Point3D(4, 3, 6), parent=world, material=wall_material)  # Walls

  # FLoor
    Box(Point3D(0, 0, 0), Point3D(4, 0.1, 6), parent=world, material=floor_material)  # Floor

  # Table
    Box(Point3D(1.5, 0, 4), Point3D(2.5, 0.5, 5), parent=world, material=table_material)


  # Sunlight through a window (east wall)
    azimuth = math.radians(90)  # Sunlight coming from the east
    elevation = math.radians(45)  # Mid-morning sun angle
    sun_direction = Vector3D(math.cos(elevation) * math.cos(azimuth), math.sin(elevation), math.cos(elevation) * math.sin(azimuth))
    sun_distance = 10  # Positioning the sun emitter outside the room
    sun_intensity = 100.0 # Sun intensity
    sun_position = Point3D(4, 1.5, 1) + sun_direction * sun_distance
    Sphere(0.5, parent=world, transform=translate(sun_position.x, sun_position.y, sun_position.z), material=UniformSurfaceEmitter(ConstantSF(sun_intensity)))
 
  # Photodiodes as flat boxes positioned inward on the table
    photodiode_material = AbsorbingSurface()
  #    photodiode_material = Lambert(ConstantSF(0.0))  # Black material for photodiodes
    diode_positions = [
        Point3D(1.6, 0.51, 4.1),  # Front-left corner, moved inward
        Point3D(2.4, 0.51, 4.1),  # Front-right corner, moved inward
        Point3D(1.6, 0.51, 4.9),  # Back-left corner, moved inward
        Point3D(2.4, 0.51, 4.9),  # Back-right corner, moved inward
    ]
    diode_size = Point3D(0.2, 0.02, 0.2)  # Width, Height (thin), Depth
    diodes = []
    for pos in diode_positions:
        lower = Point3D(pos.x - diode_size.x / 2, pos.y, pos.z - diode_size.z / 2)
        upper = Point3D(pos.x + diode_size.x / 2, pos.y + diode_size.y, pos.z + diode_size.z / 2)
        diode = Box(lower, upper, parent=world, material=photodiode_material)
        diodes.append(diode)

  # Photodiode pipelines to measure power
    photodiode_pipelines = []
    for diode in diodes:
        pipeline = PowerPipeline0D()
        Pixel(pipelines=[pipeline], parent=diode)
        photodiode_pipelines.append(pipeline)

  # Process the ray-traced spectra with the RGB pipeline.
    rgb = RGBPipeline2D()

  # Camera
 #    rgb_pipeline = RGBPipeline2D(display_progress=False) #, display_auto_exposure=True)
    camera = PinholeCamera(
        (1024, 1024),  # Increased resolution
        fov=55,
        pipelines=[rgb],
        parent=world,
        transform=translate(2, 1.5, 0.5) * rotate(0, -3, 0),
 #        transform=translate(2, 1.5, 5.5) * rotate(180, -3, 0),
    )
    camera.pixel_samples = 10  # Increace samples for better quality
    camera.min_wavelength = 100
    camera.max_wavelength = 900

  # Render and save image
    plt.ion()
    camera.observe()

    plt.ioff()
    rgb.save("lit_room_with_led_and_sunlight.png")
    rgb.display()
    plt.show()
 
    print("Rendering complete. Image saved as 'lit_room_with_led_and_sunlight.png'.")

  # Print photodiode power measurements
    for i, pipeline in enumerate(photodiode_pipelines):
        power_mean = pipeline.value.mean
        power_error = pipeline.value.error()
        print(f"Photodiode {i + 1} measured power: {power_mean:.6f} W ± {power_error:.6f} W")

create_scene()
