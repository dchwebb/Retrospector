$port = New-Object System.IO.Ports.SerialPort COM17
$port.open()
$port.WriteLine("both")
echo $port.ReadLine()
pause