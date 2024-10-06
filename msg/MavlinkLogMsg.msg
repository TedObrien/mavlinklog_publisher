# Possible levels of operations.

# byte OK=0
# byte WARN=1
# byte ERROR=2
# byte STALE=3

# Level of operation enumerated above.
uint8 level

# Severity = 6, Info - message in QGC
# Severity = 5, Notice - QGC shouts 
# Severity = 4, Warning - QGC shouts 
# severity = 3 Error - QGC shouts and pop up in qgc
# severity = 2 Critical - QGC shouts and pop up in qgc
# severity = 1 Alert - QGC shouts and pop up in qgc
# severity = 0 Emergency - QGC shouts and pop up in qgc

# Optional ID of publisher.
string id

# A description of the status.
string message

