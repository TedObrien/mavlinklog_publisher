# Message levels for different types of operations.

# Level 6: Info - Displayed as a message in QGC.
# Level 5: Notice - Displayed as a message in QGC + QGC announces.
# Level 4: Warning - Displayed as a message in QGC + QGC announces.
# Level 3: Error - Displayed as a message in QGC + QGC announces and displays pop-up.
# Level 2: Critical - Displayed as a message in QGC + QGC announces (shouts) and displays pop-up.
# Level 1: Alert - Displayed as a message in QGC + QGC announces and displays pop-up.
# Level 0: Emergency - Displayed as a message in QGC + QGC announces and displays pop-up.

# Level of operation as enumerated above.
uint8 level

# Optional ID of the publisher.
string id

# Descriptive message or status information.
string message

