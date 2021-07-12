# CMRI-Code
<h1>Code used on various arduinos, to communicate over CMRI (RS-485), for sensors outputs etc.</h1>
<p>CMRI_Turnouts_and_Sensors_servo_bounce sketch is designed to be used on an Arduino Mega
and was initially based on the code developed by the Little Wicket Railway channel on YouTube.<p>
<p>However it has evolved somewhat, and includes the facility to slow the servos down without having
to wait for them to complete their movement, as well as a simple signal bounce.</p>
<p>It will need further changes if using with more than a single point (turnout), but that will be done at a later date.</p>
<p>The other two files were used to test multiple nodes on a CMRI network, and will run on Arduino Nanos.</p>
<p>The CMRI_Turnouts_and_Sensors_Relay sketch is a further development of the CMRI_Turnouts_and_Sensors_servo_bounce sketch, for two objectives:
  <ol>
    <li>To remove the control of the frog relay from JMRI/CMRI and have it done internally on the Arduino. This was done as it made more sense to me since you would never want the frog doing some thing different to the point.</li>
    <li>To make the code easier to expand to accept more servos</li>
  </ol>
