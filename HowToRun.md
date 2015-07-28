# Introduction #

This is a short guide on how to run development version.

Clone the repository: git clone https://code.google.com/p/pybilliard/

# Prerequisites #

## pyglet ##

Download latest and install from http://code.google.com/p/pyglet/


## pyeuclid ##

Download latest and install from http://code.google.com/p/pyeuclid/


## bullet ##

Download latest, compile and install from http://code.google.com/p/bullet/


## pybullet ##

Currently the pybullet version used is fork and must be compiled and installed from the cloned sources. <br>
Assume that source is cloned in pybilliard folder, then run compile from 'pybilliard/pybullet' folder:<br>
<pre><code>python setup.py build_ext --include-dirs /usr/local/include/bullet/:. --library-dirs /usr/local/lib/<br>
</code></pre>

<ul><li>Require latest Cython (tested with version 0.15.1)<br>
</li><li>Note the current folder '.' added to the --include-dirs path for additional header file btUserMotionState.h<br>
</li><li>In case the bullet is installed in other folder than '/usr/local' then specify in --include-dirs and --library-dirs paths.</li></ul>

Then install with command:<br>
<pre><code>python setup.py install <br>
</code></pre>


<h1>Running test</h1>

After all prerequisites are satisfied run test.py from 'pybilliard/lib' folder:<br>
<pre><code>python test.py<br>
</code></pre>