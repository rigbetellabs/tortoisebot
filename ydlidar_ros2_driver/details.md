# ROS Paramters Table

## Dataset
<table>
<tr><th>LIDAR      <th> Model  <th>  Baudrate <th>  SampleRate(K) <th> Range(m)  		<th>  Frequency(HZ) <th> Intenstiy(bit) <th> SingleChannel<th> voltage(V)
<tr><th> F4        <td> 1	   <td>  115200   <td>   4            <td>  0.12~12         <td> 5~12           <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> S4        <td> 4	   <td>  115200   <td>   4            <td>  0.10~8.0        <td> 5~12 (PWM)     <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> S4B       <td> 4/11   <td>  153600   <td>   4            <td>  0.10~8.0        <td> 5~12(PWM)      <td> true(8)        <td> false    	  <td> 4.8~5.2
<tr><th> S2        <td> 4/12   <td>  115200   <td>   3            <td>  0.10~8.0     	<td> 4~8(PWM)       <td> false          <td> true    	  <td> 4.8~5.2
<tr><th> G4        <td> 5	   <td>  230400   <td>   9/8/4        <td>  0.28/0.26/0.1~16<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> X4        <td> 6	   <td>  128000   <td>   5            <td>  0.12~10     	<td> 5~12(PWM)      <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> X2/X2L    <td> 6	   <td>  115200   <td>   3            <td>  0.10~8.0     	<td> 4~8(PWM)       <td> false          <td> true    	  <td> 4.8~5.2
<tr><th> G4PRO     <td> 7	   <td>  230400   <td>   9/8/4        <td>  0.28/0.26/0.1~16<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> F4PRO     <td> 8	   <td>  230400   <td>   4/6          <td>  0.12~12         <td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> R2        <td> 9	   <td>  230400   <td>   5            <td>  0.12~16         <td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G6        <td> 13     <td>  512000   <td>   18/16/8      <td>  0.28/0.26/0.1~25<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G2A       <td> 14	   <td>  230400   <td>   5            <td>  0.12~12         <td> 5~12      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G2        <td> 15     <td>  230400   <td>   5            <td>  0.28~16     	<td> 5~12      	    <td> true(8)        <td> false    	  <td> 4.8~5.2
<tr><th> G2C       <td> 16	   <td>  115200   <td>   4            <td>  0.1~12        	<td> 5~12      	    <td> false      	<td> false    	  <td> 4.8~5.2
<tr><th> G4B       <td> 17	   <td>  512000   <td>   10           <td>  0.12~16         <td> 5~12        	<td> true(10)       <td> false    	  <td> 4.8~5.2
<tr><th> G4C       <td> 18	   <td>  115200   <td>   4            <td>  0.1~12		    <td> 5~12           <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G1        <td> 19	   <td>  230400   <td>   9            <td>  0.28~16         <td> 5~12      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G5        <td> 20	   <td>  230400   <td>   9/8/4        <td>  0.28/0.26/0.1~16<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G7        <td> 21         <td>  512000   <td>   18/16/8      <td>  0.28/0.26/0.1~25<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> TX8    　 <td> 100	   <td>  115200   <td>   4            <td>  0.05~8      	<td> 4~8(PWM)       <td> false          <td> true      	  <td> 4.8~5.2
<tr><th> TX20    　<td> 100	   <td>  115200   <td>   4            <td>  0.05~20      	<td> 4~8(PWM)       <td> false          <td> true     	  <td> 4.8~5.2
<tr><th> TG15    　<td> 100	   <td>  512000   <td>   20/18/10     <td>  0.05~30      	<td> 3~16      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> TG30    　<td> 101	   <td>  512000   <td>   20/18/10     <td>  0.05~30      	<td> 3~16      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> TG50    　<td> 102	   <td>  512000   <td>   20/18/10     <td>  0.05~50      	<td> 3~16      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> T15     　<td> 200	   <td>  8000     <td>   20           <td>  0.05~30      	<td> 5~35      	    <td> true           <td> false    	  <td> 4.8~5.2
</table>

## Baudrate Table

| LiDAR                					| baudrate               | 
|-----------------------------------------------|-----------------------|
|F4/S2/X2/X2L/S4/TX8/TX20/G4C 		| 115200			|
|X4                   					| 128000			|
|S4B                         				| 153600			|
|G1/G2/R2/G4/G5/G4PRO/F4PRO         	| 230400			|
|G6/G7/TG15/TG30/TG50			 	| 512000			|
|T5/T15			 	                | 8000 (network port)			|


## SingleChannel Table

| LiDAR                							| isSingleChannel       | 
|-----------------------------------------------------------|-----------------------|
|G1/G2/G4/G5/G6/G7/F4/F4PRO/S4/S4B/X4/R2/G4C 	| false			|
|S2/X2/X2L                   					| true			|
|TG15/TG30/TG50                         		| false			|
|TX8/TX20         							    | true			|
|T5/T15        							    | false	(optional)		|


## LidarType Table

| LiDAR                									| lidar_type             | 
|-----------------------------------------------------------------------|-----------------------|
|G1/G2/G4/G5/G6/G7/F4/F4PRO/S4/S4B/X4/R2/G4C/S2/X2/X2L 	| TYPE_TRIANGLE			|
|TG15/TG30/TG50/TX8/TX20                   				| TYPE_TOF			|
|T5/T15                 				                | TYPE_TOF_NET			|

## DeviceType Table

| LiDAR                									| lidar_type             | 
|-----------------------------------------------------------------------|-----------------------|
|G1/G2/G4/G5/G6/G7/F4/F4PRO/S4/S4B/X4/R2/G4C/S2/X2/X2L 	| YDLIDAR_TYPE_SERIAL			|
|TG15/TG30/TG50/TX8/TX20                   				| YDLIDAR_TYPE_SERIAL			|
|T5/T15                 				                | YDLIDAR_TYPE_TCP			|


## Sampling Rate Table

| LiDAR                		| sample_rate             | 
|-----------------------------|------------------------|
|G4/G5/F4                   | 4,8,9			 |
|F4PRO                   	| 4,6   	     |
|G6/G7                      | 8,16,18		 |
|G2/R2/X4         		    | 5				 |
|G1 	                    | 9			 	 |
|S4/S4B/G4C/TX8/TX20 	    | 4			 	 |
|S2                    		| 3			 	 |
|TG15/TG30/TG50             | 10,18,20		 |
|T5/T15                     | 20		     |


## Frequency Table

| LiDAR                					| frequency             | 
|-----------------------------------------------|------------------------|
|G1/G2/R2/G6/G7/G4/G5/G4PRO/F4/F4PRO	| 5-12Hz			 |
|S4/S4B/S2/TX8/TX20/X4 			        | Not Support		 |
|TG15/TG30/TG50           			    | 3-16Hz			 |
|T5/T15           			            | 5-35Hz			 |

Note: For unsupported LiDARs, adjusting the scanning frequency requires external access to PWM speed control.

## Reversion Table
 <table>
      <tr><th>LiDAR                           <th>reversion
      <tr><th>G1/G2/G2A/G2C/F4/F4PRO/R2       <td>true
      <tr><th>G4/G5/G4PRO/G4B/G4C/G6/G7       <td>true
      <tr><th>TG15/TG30/TG50                  <td>true
      <tr><th>T5/T15                          <td>true
      <tr><th>S2/X2/X2L/X4/S4/S4B             <td>false
      <tr><th>TX8/TX20                        <td>false
  </table>

## Intensity Table
 <table>
    <tr><th>LiDAR                                    <th>intensity
      <tr><th>S4B/G2/G4B                             <td>true
      <tr><th>G4/G5/G4C/G4PRO/F4/F4PRO/G6/G7         <td>false
      <tr><th>G1/G2A/G2C/R2                          <td>false
      <tr><th>S2/X2/X2L/X4                           <td>false
      <tr><th>TG15/TG30/TG50                         <td>false
      <tr><th>TX8/TX20                               <td>false
      <tr><th>T5/T15                                 <td>true
      <tr><th>                                       <td>false
  </table>

## DTR Support Table

 <table>
      <tr><th>LiDAR                                  <th>support_motor_dtr
      <tr><th>S4/S4B/S2/X2/X2L/X4                    <td>true
      <tr><th>TX8/TX20                               <td>true
      <tr><th>G4/G5/G4C/G4PRO/F4/F4PRO/G6/G7         <td>false
      <tr><th>G1/G2A/G2C/R2/G2/G4B                   <td>false
      <tr><th>TG15/TG30/TG50                         <td>false
      <tr><th>T5/T15                                 <td>false
  </table>
