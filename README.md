This is my science fair project from 2018.  It was originally written in Python 3 and I am porting it to F# as an exercise.



I wrote it becasue I wanted to simulate a rocket launch from Earth.  
The scientific purpose was to see if an alternative manned Mars exploration mission architecture called: "Mars Direct" would be acheivable with planned launchers (2018 plans).  A modified space shuttle fuel tank with the shuttle's engines and boosters was the plan in the early 90's.  Since the 90's, the shuttle program ended and the US has had no heavy lift capabialties.  The Space Launch System (which was recently dealt a huge blow in funding) is NASA's new plan and my simuation uses the proposed specs that were available in 2017-18. 

The program is broken into 2 files and ~5 parts:

The first part establishes all of the starting variables and constants such as the mass of the earth and the rocket's position on the surface of the Earth with 0 motion.  It also makes a bunch of empty vectors to store "telemetry" in for later analysis and graphing.  
The second part takes the rocket's current positon and gives all of the atmospheric qualities that the core simulation will use in it's calculations.  
The Third part is the core of the simulation and it performs all of the math involving thrust, drag, gravity, velocity, and position.  
The Fourth part takes all of the data stored in the vectors and makes graphs so that we can see how the flight went after the simuation has finished its last cycle.  
The fifth part is in a seperate file and it stores the specs for each payload and launcher; as well as the funtions for the rocket's physical specs over time such as mass, drag, and thrust as the rocket drops stages and consumes fuel.  

More details
The simulation only uses 2 physical dimensions becasue it is only testing payload capacity and not guidance software.  
