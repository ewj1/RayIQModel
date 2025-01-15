import matplotlib.pyplot as plt
from raysect.primitive import Box, Cylinder
from raysect.optical import World, Point3D, Vector3D, translate, rotate, rotate_x, rotate_z, Material
from raysect.optical.material import Lambert, UniformSurfaceEmitter, AbsorbingSurface
from raysect.optical.observer import PinholeCamera, Pixel
from raysect.optical.observer.pipeline import RGBPipeline2D, PowerPipeline0D
from raysect.optical.spectralfunction import ConstantSF
import math

# Initialize the world
world = World()

scale_factor = 200  # Scaling factor for the sensor assembly

# Materials
ceiling_material = Lambert(ConstantSF(0.85))
wall_material = Lambert(ConstantSF(0.9))
floor_material = Lambert(ConstantSF(0.7))
table_material = Lambert(ConstantSF(0.1))# make table light colored
metallic_wall_material = Lambert(ConstantSF(0.5)) # edit for visual validatation - I can't see the walls (was 0.85)
#photodiode_material = Lambert(ConstantSF(1.0))    # allows visual inspection of shadows (looks white)
photodiode_material = AbsorbingSurface()                    #  measures power (looks black)

# Table
table_center = Point3D(2, 0, 4)  # Center of the table (FIXED centerpoint of experiment)

tablex = .00025 * scale_factor # width
tabley = .0000025 * scale_factor # height
tablez = .000125 * scale_factor # length

table = Box(Point3D(-tablex/2, -tabley, -tablez/2,), Point3D(tablex/2, 0, tablez/2,),
            parent=world, transform=translate(table_center.x, table_center.y, table_center.z), material=table_material) 


rotate_sensor = 0  # adjust 90 to see along wall, 0 to see across wall 
print(f"Sensor rotation in degrees: {rotate_sensor}")

# Emission intensity for LEDs
emission_intensity = 50000000
led_material = UniformSurfaceEmitter(ConstantSF(emission_intensity))
led_diameter = 0.0024 # supposed to be 2.4 mm
led_diameter *= scale_factor
led_radius = led_diameter / 2
led_thickness = 0.05

# LED Parameters
led_angle_primary = 45  # Rotation away from Z-axis (degrees) azimuth
led_angle_secondary = -10  # Rotation away from Y-axis (degrees) elevation
source_to_sensor_distance = 0.5  # Distance from LED to table center (meters)

# Calculate Cartesian coordinates of the LED center (not normal spherical conversion, adjusted for y as vertical axis)
ledz = 0 + source_to_sensor_distance * math.sin(math.radians(led_angle_secondary) * math.cos(math.radians(led_angle_primary)))
ledx = 0 + source_to_sensor_distance * math.sin(math.radians(led_angle_secondary) * math.sin(math.radians(led_angle_primary)))
ledy = 0 + source_to_sensor_distance * math.cos(math.radians(led_angle_secondary))
led_center = Point3D(ledx, ledy, ledz)

delta_x = 0 - led_center.x
delta_y = 0 - led_center.y
delta_z = 0 - led_center.z

yaw = 90+math.degrees(math.atan2(delta_z, delta_x))  # rotation in the xz plane (away from z axis)
pitch = -math.degrees(math.atan2(delta_y, math.sqrt(delta_x**2 + delta_z**2)))
roll = 0  # Circle symmetry

print(f"LED Center Coordinates (XYZ): {led_center}")
print(f"LED Rotation Angles - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

# Construct the LED emitter
led_emitter = Cylinder(
    led_radius, 
    led_thickness, 
    transform=translate(ledx, ledy, ledz) * rotate(roll, pitch, yaw), 
    parent=world, 
    material=led_material
)

# Function to build the sensor assembly
def build_sensor(world, table_center, scale_factor):
    """
    Builds scaled LDD and LQD sensor cells, placing them with a specified gap between LDD and LQD, centered on the table.

    :param world: The root node of the scene.
    :param table_center: The center coordinates of the table as a Point3D object.
    :param scale_factor: The factor by which to scale the dimensions of the assembly.
    """
    # Base dimensions at actual size (in meters)
    ratio = 2  # photodiode width RATIO to wall height-6
    wall_length = 100e-6  # 100 microns
    wall_height = 5e-6    # 5 microns
    wall_thickness = 1e-6 # 1/2 micron
    t_top_overhang = 2e-6 # 5 microns
    t_top_thickness = 1e-6 # 1/2 micron
    diode_length = 80e-6  # 80 microns
    diode_width = (wall_height * ratio) #10e-6   # 10 microns
    diode_thickness = 1e-6 # 1 micron
    wall_gap = 10e-6       # 6 microns
    

    # Apply scaling
    wall_length *= scale_factor
    wall_height *= scale_factor
    wall_thickness *= scale_factor
    t_top_overhang *= scale_factor
    t_top_thickness *= scale_factor
    diode_length *= scale_factor
    diode_width *= scale_factor
    diode_thickness *= scale_factor
    wall_gap *= scale_factor

    # Calculate positions
    half_gap = wall_gap / 2
    ldd_position = table_center.x - half_gap - wall_length /2  # LDD placed to the left
    lqd_position = table_center.x + half_gap + wall_length /2  # LQD placed to the right

    # LDD Wall
    ldd_wall = Box(
        Point3D(-wall_length / 2, 0, -wall_thickness / 2),
        Point3D(wall_length / 2, diode_thickness + wall_height, wall_thickness / 2),
        parent=world,
        material=metallic_wall_material,
        transform=translate(ldd_position, table_center.y, table_center.z) * rotate(rotate_sensor, 0, 0)  # Rotate 90 degrees around Y-axis
    )

    # LQD Wall with T-top
    lqd_wall_vertical = Box(
        Point3D(-wall_length / 2, 0, -wall_thickness / 2),
        Point3D(wall_length / 2, diode_thickness + wall_height, wall_thickness / 2),
        parent=world,
        material=metallic_wall_material,
        transform=translate(lqd_position, table_center.y, table_center.z) * rotate(rotate_sensor, 0, 0)  # Rotate 90 degrees around Y-axis
    )

    lqd_wall_top = Box(
        Point3D(-wall_length / 2, diode_thickness + wall_height - t_top_thickness, - t_top_overhang - wall_thickness / 2),
        Point3D(wall_length / 2, diode_thickness + wall_height                   ,+ t_top_overhang + wall_thickness / 2),
        parent=world,
        material=metallic_wall_material,
        transform=translate(lqd_position, table_center.y, table_center.z) * rotate(rotate_sensor, 0, 0)  # Rotate 90 degrees around Y-axis
    )


    # Photodiode positions relative to the walls
    diode_positions = [
        Point3D(ldd_position, table_center.y, table_center.z - (diode_width /2) - (wall_thickness / 2)),  # bottom-left  (LDD)
        Point3D(ldd_position, table_center.y, table_center.z + (diode_width /2) + (wall_thickness / 2)),  # top-left     (LDD)
        Point3D(lqd_position, table_center.y, table_center.z - (diode_width /2) - (wall_thickness / 2)),  # bottom-right (LQD)
        Point3D(lqd_position, table_center.y, table_center.z + (diode_width /2) + (wall_thickness / 2))   # top-right    (LQD)
    ]

    # Create photodiodes
    diodes = []
    for pos in diode_positions:
        lower = Point3D(-diode_length / 2, -diode_thickness / 2, -diode_width / 2)
        upper = Point3D(diode_length / 2, diode_thickness / 2, diode_width / 2)
        diode = Box(lower, upper, parent=world, material=photodiode_material,
                    transform=translate(pos.x, pos.y, pos.z) * rotate(0, rotate_sensor, 0))  # Rotate 90 degrees around Y-axis)
        diodes.append(diode)
        
    # Photodiode pipelines to measure power
    photodiode_pipelines = []
    photodiode_observers = []
    for diode in diodes:
        pipeline = PowerPipeline0D()
        observer = Pixel(pipelines=[pipeline], x_width = diode_width, y_width = diode_length,
                        pixel_samples =5*10**6,
                        transform = translate(0,diode_length+0.01, 0)*rotate(-90,0,0), parent=diode)
        photodiode_pipelines.append(pipeline)
        photodiode_observers.append(observer)

    return diodes, photodiode_pipelines, photodiode_observers, diode_width, diode_length

# Usage
#table_center = Point3D(2, 0, 4)  # Center of the table

diodes, photodiode_pipelines, photodiode_observers, diode_width, diode_length = build_sensor(world, table_center, scale_factor)

# Process the ray-traced spectra with the RGB pipeline.
rgb1 = RGBPipeline2D(display_auto_exposure=True)
rgb2 = RGBPipeline2D(display_auto_exposure=True)
rgb3 = RGBPipeline2D(display_auto_exposure=True)
rgb_pipelines = [rgb1, rgb2, rgb3]

# Print the XYZ coordinates of the table center and LED center
print(f"Table center coordinates (XYZ): {table_center}")
print(f"LED center coordinates (XYZ): {led_center}")

x_angle = math.degrees(math.atan2(delta_y, math.sqrt(delta_x**2 + delta_z**2)))
y_angle = math.degrees(math.atan2(delta_z, delta_x))
roll_angle = math.degrees(math.atan2(delta_z, delta_y))

print(f"Bearing from table to LED:")
print(f"  X angle: {x_angle:.2f} degrees")
print(f"  Y angle: {y_angle:.2f} degrees")
print(f"Roll angle: {roll_angle:.2f} degrees")

magnitude = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
unit_vector = (delta_x / magnitude, delta_y / magnitude, delta_z / magnitude)

# Calculate roll angle (3D bearing)
roll_angle = math.degrees(math.atan2(delta_y, delta_z))

# Output the roll angle and unit vector
print(f"Roll angle: {roll_angle:.2f} degrees")
print(f"Unit vector from table to LED: {unit_vector}")


# Measure power with photodiodes
for observer in photodiode_observers:
    observer.observe()

# Print photodiode power measurements
for i, pipeline in enumerate(photodiode_pipelines):
    power_mean = pipeline.value.mean
    power_error = pipeline.value.error()
    print(f"Photodiode {i + 1} measured power: {power_mean:.6f} W ± {power_error:.6f} W")

# Camera
camera1 = PinholeCamera(
    (512, 512),  # Resolution
    fov=30,    #was 55,
    pipelines=[rgb1],
    parent=world,
#  transform=translate(2, 1.75, 5.5) * rotate(180, -5, 0), # whole scene
#   transform=translate(2, .75, 2) * rotate(180, -20, 0),  # camera close-up
   transform=translate(2, 0.5, 3.2) * rotate(0, -30, 0),  # camera closer-upper
#    transform=translate(3, 1, 3.5) * rotate(90, -20, 0),  # attempt to look down the lenght of the walls (needs work)
)
camera1.pixel_samples = 1000  # Increase samples for better quality
camera1.min_wavelength = 550
camera1.max_wavelength = 750


# Camera 2: Down the sensor walls
camera2 = PinholeCamera(
    (512, 512), 
    fov=45,  # Adjusted FOV to see both sensor cells
    pipelines=[rgb2],
    transform=translate(2.5, 0.6, 4) * rotate(90, -45, 0),  # Aligned along sensor walls
    parent=world
)

camera2.pixel_samples = 1000  # Increase samples for better quality
camera2.min_wavelength = 550
camera2.max_wavelength = 750

# Calculate the position 20% of the way from the LED to the table center
percntd = 0.8
camera_position3 = Point3D(
    led_center.x + percntd * (table_center.x - led_center.x),
    led_center.y + percntd * (table_center.y - led_center.y),
    led_center.z + percntd * (table_center.z - led_center.z),
)

# Compute the angles for rotation
distance_xy = math.sqrt(delta_x**2 + delta_y**2)
pitch = math.degrees(math.atan2(delta_z, distance_xy))  # Rotation about X-axis
yaw = math.degrees(math.atan2(delta_y, delta_x))        # Rotation about Z-axis
roll = 180  # No roll for this setup

# Create camera3  with the computed position and orientation
camera3 = PinholeCamera(
    (512, 512),  # Resolution
    fov=100,  # Adjusted FOV
    pipelines=[rgb3],  # RGB pipeline
    parent=world,
    transform=translate(camera_position3.x, camera_position3.y, camera_position3.z) *
              rotate(pitch, yaw, roll)  # Transform: Translation + Rotation
)

# Debug print to verify camera setup
print(f"Camera 3 Position: {camera_position3}")
print(f"Camera 3 Rotation - Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, Roll: {roll:.2f}")


camera3.pixel_samples = 1000  # Increase samples for better quality
camera3.min_wavelength = 550
camera3.max_wavelength = 750

print(f"Camera 3 Rotation - Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, Roll: {roll:.2f}")


cameras = [camera1, camera2, camera3]
camera_names = ["camera1", "camera2", "camera3"]

# Rendering process
try:
    plt.ion()  # Enable interactive plotting
    for i, camera in enumerate(cameras):
        print(f"Rendering {camera_names[i]}...")
        camera.observe()
        rgb_pipelines[i].display()  # Use the correct RGB pipeline for each camera
        plt.pause(0.1)  # Allow rendering progress to update
        rgb_pipelines[i].save(f"{camera_names[i]}_view.png")
        print(f"{camera_names[i]} view saved as '{camera_names[i]}_view.png'.")
    plt.ioff()  # Disable interactive plotting
except KeyboardInterrupt:
    print("\nRendering interrupted by user. Exiting gracefully.")
