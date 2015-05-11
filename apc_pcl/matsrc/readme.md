# Better comparison
- We need a smarter way to compare things. If a distribution is shifted, we lose some happiness


# Using these files in Python

import pymatlab
session = pymatlab.session_factory()
import numpy as np

a = np.random.randn(20, 10, 30)
session.putvalue('a', a)  # Create a variable in the matlab workspace called a
session.run('f = sqrt('a.^2);')
f = session.getvalue('f')