# Possible levels of operations.

# byte OK=0
# byte WARN=1
# byte ERROR=2
# byte STALE=3

# Level of operation enumerated above.
uint8 level

# Severity = 6, info message in QGC
# Severity = 5, QGC shouts and notice message in QGC
# Severity = 4, QGC shouts message but no pop up
# severity = 3 QGC shouts and error pop up in qgc
# severity = 2 QGC shouts and ciritcal pop up in qgc
# severity = 1 QGC shouts and alert pop up in qgc
# severity = 0 

# Optional ID of publisher.
string id

# A description of the status.
string message

