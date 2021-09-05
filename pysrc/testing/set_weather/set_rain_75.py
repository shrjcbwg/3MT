import setup_path
import airsim
import time

client = airsim.MultirotorClient()


client.simEnableWeather(True)
client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0.75)
client.simSetWeatherParameter(airsim.WeatherParameter.Roadwetness, 0.75)
airsim.wait_key('Press any key to reset weather')
client.simSetWeatherParameter(airsim.WeatherParameter.Roadwetness, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0)
# client.simPrintLogMessage("Setting Snow to 0%")