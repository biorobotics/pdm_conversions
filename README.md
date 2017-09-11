# pdm_conversions

This catkin package was created to correct the GOAT_messages from dataset2.
- Places missing timestamp in new message (pre-recorded .csv file of timestamp that was put externally instead of internally to messages)
- Re-emits messages with correct type (instead of custom formats, standard formats)
- Mix Imperial/metric to 100% metric conversion
- The problem of push_back and push_front of the original array custom formats is taken care of
- grouped goat topics names, now all starting with goat/<name>

Note that as wait/sleep was used in the goat message emitting code, and hence the messages are not continious especially during the "sit-stand" procedure, or might completely be missing.


See launch file for more information how to use this very specific node. 
