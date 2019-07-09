# Perceptbot State Machine

The state machine for perceptbot is created using the Smach state machine library:
http://wiki.ros.org/smach

State classes are defined as follows:

```python
class Foo(smach.State):
     def __init__(self, outcomes=['outcome1', 'outcome2'],
                        input_keys=['foo_input'],
                        output_keys=['foo_output'])

     def execute(self, userdata):
        # Do something with userdata
        if userdata.foo_input == 1:
            return 'outcome1'
        else:
            userdata.foo_output = 3
            return 'outcome2'
```

The outcomes are the states that the currently executing state can transition into.

State I/O is performed with the `_input_keys` and `_output_keys` which can be accessed during execution using `userdata`.
