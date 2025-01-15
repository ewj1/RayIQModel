import matplotlib.pyplot as plt
from raysect.primitive import Box, Cylinder, Sphere
from raysect.optical import World, Point3D, Vector3D, translate, rotate, rotate_x,rotate_y, rotate_z, Material
from raysect.optical.material import Lambert, UniformSurfaceEmitter, AbsorbingSurface
from raysect.optical.observer import PinholeCamera, Pixel
from raysect.optical.observer.pipeline import RGBPipeline2D, PowerPipeline0D
from raysect.optical.spectralfunction import ConstantSF
import math

# Initialize the world
world = World()

#Box(Point3D(-0.5,-0.5,-0.5), Point3D(0.5,0.5,0.5), transform = translate(0,0,0), material = Lambert(ConstantSF(1)), parent = world)
Cylinder(radius = 0.05, height=1, material = Lambert(ConstantSF(1)), parent = world) #+ve z-axis
Cylinder(radius = 0.05, height=1, transform = rotate_y(90), material = Lambert(ConstantSF(1)), parent = world) #+ve x-axis
Cylinder(radius = 0.05, height=1, transform = rotate_x(-90), material = Lambert(ConstantSF(1)), parent = world) #+ve y-axis
Sphere(radius = 0.5, parent = world, transform = translate(0,3,-3), material = UniformSurfaceEmitter(ConstantSF(10)))
Sphere(radius = 0.5, parent = world, transform = translate(0,3,3), material = UniformSurfaceEmitter(ConstantSF(10)))


# LED Parameters
led_angle_primary = 45  # Rotation around X-axis (degrees)
led_angle_secondary = 0  # Rotation around Z-axis (degrees)
source_to_sensor_distance = 0.5  # Distance from LED to table center (meters)

# Calculate Cartesian coordinates of the LED center
ledx = 0 + source_to_sensor_distance * math.cos(math.radians(led_angle_secondary)) * math.sin(math.radians(led_angle_primary))
ledy = 0 + source_to_sensor_distance * math.sin(math.radians(led_angle_primary))
ledz = 0 + source_to_sensor_distance * math.cos(math.radians(led_angle_primary)) * math.sin(math.radians(led_angle_secondary))

led_center = Point3D(ledx, ledy, ledz)

# Calculate the rotations needed to point the LED at the table center
delta_x = 0 - led_center.x
delta_y = 0 - led_center.y
delta_z = 0 - led_center.z

pitch = -math.degrees(math.atan2(delta_y, math.sqrt(delta_x**2 + delta_z**2)))  # Rotation around X-axis
yaw = math.degrees(math.atan2(delta_z, delta_x))  # Rotation around Y-axis
roll = 0  # Assuming no roll is needed

print(f"LED Center Coordinates (XYZ): {led_center}")
print(f"LED Rotation Angles - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

# Construct the LED emitter
led_radius = 0.12  # 24 cm diameter
led_thickness = 0.05  # Thin disk
point_back = Cylinder(
    0.05, 
    1, 
    transform=translate(ledx, ledy, ledz) * rotate(roll, pitch, yaw), 
    parent=world, 
    material=Lambert(ConstantSF(1))
)




# Process the ray-traced spectra with the RGB pipeline.
rgb = RGBPipeline2D(display_auto_exposure=False)
rgb_pipeline = [rgb]

# Camera
camera1 = PinholeCamera(
    (512, 512),  # Resolution
    fov=60,    #was 55,
    pipelines=[rgb],
    parent=world,
    transform=translate(0, 1, 4) * rotate_y(180)
)
camera1.pixel_samples = 100  # Increase samples for better quality
camera1.min_wavelength = 550
camera1.max_wavelength = 750

cameras = [camera1]
camera_names = ["camera1"]

# Rendering process
try:
    plt.ion()  # Enable interactive plotting
    for i, camera in enumerate(cameras):
        print(f"Rendering {camera_names[i]}...")
        camera.observe()
        rgb_pipeline[i].display()  # Use the correct RGB pipeline for each camera
        plt.pause(0.1)  # Allow rendering progress to update
        rgb_pipeline[i].save("original_orientation_view.png")
        print(f"{camera_names[i]} view saved as 'original_orientation_view.png'.")
    plt.ioff()  # Disable interactive plotting
except KeyboardInterrupt:
    print("\nRendering interrupted by user. Exiting gracefully.")