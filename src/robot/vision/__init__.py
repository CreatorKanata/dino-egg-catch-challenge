"""src/robot/vision/__init__.py: Front-camera perception for Auto Catch (parent process only).

egg_size.py and timing.py are stdlib-only and safe everywhere (the signboard child imports
egg_size through display_status). The other modules use numpy and OpenCV; OpenCV is imported
lazily inside functions, so importing this package never loads cv2 by itself. The signboard child
must never call them (opencv and pygame each bundle libSDL2 on macOS).
"""
