"""src/robot/vision/__init__.py: Front-camera perception for Auto Catch and Auto Release (parent process only).

egg_size.py, basket_size.py, config_vision.py, and timing.py are stdlib-only and safe everywhere
(the signboard child imports egg_size and basket_size through display_status). The other modules use numpy and OpenCV; OpenCV is imported
lazily inside functions, so importing this package never loads cv2 by itself. The signboard child
must never call them (opencv and pygame each bundle libSDL2 on macOS).
"""
