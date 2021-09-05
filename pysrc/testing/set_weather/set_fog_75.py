import setup_path
import airsim
import time

client = airsim.MultirotorClient()


client.simEnableWeather(True)
client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0.75)
airsim.wait_key('Press any key to reset weather')
client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0)
# client.simPrintLogMessage("Setting Snow to 0%")