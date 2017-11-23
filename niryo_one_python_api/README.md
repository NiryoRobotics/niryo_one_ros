# Niryo One Python API

To use Python API :

1. Connect to Niryo One via ssh

2. Create a Python file 

```
touch test_python_api.py
chmod +x test_python_api.py
```

3. Use this template to write your code

```python
#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
import rospy
import time
rospy.init_node('niryo_one_example_python_api')

n = NiryoOne()

try:
    # Your code here
except NiryoOneException as e:
    print e 
    # Handle errors here
```
