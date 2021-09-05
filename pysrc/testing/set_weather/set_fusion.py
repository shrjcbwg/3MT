
import airsim
import time

client = airsim.MultirotorClient()


client.simEnableWeather(True)
client.simSetWeatherParameter(airsim.WeatherParameter.Snow, 1)
client.simSetWeatherParameter(airsim.WeatherParameter.RoadSnow, 1)
client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 1)
client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 1)
client.simSetWeatherParameter(airsim.WeatherParameter.Roadwetness, 1)
airsim.wait_key('Press any key to reset weather')
client.simSetWeatherParameter(airsim.WeatherParameter.RoadSnow, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.Snow, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.Roadwetness, 0)
# client.simPrintLogMessage("Setting Snow to 0%")