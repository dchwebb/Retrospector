$port = New-Object System.IO.Ports.SerialPort COM17
$port.open()
$port.WriteLine("pp")
echo $port.ReadLine()
$port.WriteLine("both")
echo $port.ReadLine()
$port.Close()
#pause