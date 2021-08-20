# CI Statuses

Check | Status
---|---
| Documentation     | [![Documentation Status](https://readthedocs.org/projects/sr-visualization/badge/?version=latest)](http://sr-visualization.readthedocs.org) |
Build|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiYWkxaldQWDhzajBCYVNkVjIrRitmUkVFR0dEbXJzRkM0VU1ocGx6cFlSbEtXZlJ4VS9nck9aS1R6ckFScFlEUnVXVWVVY2NHVnVPOU5RVEFWampmNFcwPSIsIml2UGFyYW1ldGVyU3BlYyI6Ii9pb1dWZC9uZVRjWUNnZjQiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=noetic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_sr_visualization_common_noetic-devel_install_check/)
Style|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiVWM2alhkQkxLSFREd2I1dDJIMXRUcUFLTkwrcTdvVkVKTnlRbkx1SUkwVkhmMmIwSjNmNHVBWTRET2xrYkVCMEtVbHY4bi9qdXdzMUloVU9SK2ZrMXEwPSIsIml2UGFyYW1ldGVyU3BlYyI6IkVDbyt5YnlXa3Mwb0JrMCsiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=noetic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_sr_visualization_common_noetic-devel_style_check/)
Code Coverage|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiOHpSeHVQTUVsOGJrYTVNeTJma0NvcU1nQlowUkRXYXMzUExobEdPVStYeW9XQnJ0ZTUyQ3d0NEJwenM3cFdmdlVzYVBDQVJBRkVBRWQ5ekhvbDdSeVVjPSIsIml2UGFyYW1ldGVyU3BlYyI6Ik9LS2JoSm9tUnNENHVnYUEiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=noetic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_sr_visualization_common_noetic-devel_code_coverage/)

# sr-visualization

This is a package with GUI plugins which can be used to control the different nodes of the shadow_robot stack. The GUI plugins are programmed in Python and can be started from within rqt

## Note
If the hand is started with a namespace, `joint_slider.py` will not work out of the box. To fix it, you need to change `self.hand_namespace` in the file to your hand's namespace. for example: `self.hand_namespace = "/hand/"`
