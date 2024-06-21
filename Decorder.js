function Decoder(b, port) {
  
  var temp = b[0]; // receive temperatre in range [0, 255], requires a fridge test
  /*if (temp > 127){
    temp = temp - 256; 
  }*/
  var gas = b[1] * 1000; //kOhm air resistance, for air pollution
  var humidity = b[2]; //humidity in percentage
  var pressure = 870 + b[3]; //870 mb is record lowest pressure, and end device sends difference between current pressure and 870 mb
  return {
    temperature: temp,
    gas: gas,
    humidity: humidity,
    pressure: pressure
  }
}
