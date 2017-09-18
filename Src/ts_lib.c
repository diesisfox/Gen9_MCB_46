#include "ts_lib.h"
#include "thermistor.h"

#define CHANNEL_COUNT 5
#define ACCEL_CAHNNEL 0
#define REGEN_CAHNNEL 1
#define INTERNAL_CAHNNEL 2
#define MOT_CAHNNEL 3
#define DRV_CAHNNEL 4

static ADC_HandleTypeDef* hadc;

static SemaphoreHandle_t tempMtxHandle;

static uint32_t aggregate[CHANNEL_COUNT];
static uint32_t sampleCount[CHANNEL_COUNT];
static uint16_t dmaBuffer[CHANNEL_COUNT];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(xSemaphoreTakeFromISR(tempMtxHandle, NULL)){
		for(int i=0; i < CHANNEL_COUNT; i++){
			aggregate[i] += dmaBuffer[i];
			sampleCount[i]++;
			if(aggregate[i] >= 0xFFFFF000){
				aggregate[i] /= sampleCount[i];
				sampleCount[i] = 1;
			}
		}
		xSemaphoreGiveFromISR(tempMtxHandle, NULL);
	}
	HAL_ADC_Stop_DMA(hadc);
	HAL_ADC_Start_DMA(hadc, (uint32_t*)dmaBuffer, CHANNEL_COUNT); //dw about ptr types. NEVER dma more than sequenced!
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc){
	Serial2_writeBuf("ADC ERROR!");
}

void Temp_begin(ADC_HandleTypeDef* hadc_in){
	for(uint8_t i=0; i<CHANNEL_COUNT; i++){
		aggregate[i]=0;
		sampleCount[i]=0;
	}
	hadc = hadc_in;
	tempMtxHandle = xSemaphoreCreateMutex();
	HAL_ADC_Start_DMA(hadc, (uint32_t*)dmaBuffer, CHANNEL_COUNT); //dw about ptr types. NEVER dma more than sequenced!
}

uint16_t getReading(uint8_t channel){
	static uint32_t n, c, s;
	if(channel < TEMP_CHANNELS && sampleCount[channel]/TEMP_OVERSAMPLING){
		n = aggregate[channel];		//aggregated sum of samples
		c = sampleCount[channel];	//total number of samples aggreagated
		s = dmaBuffer[channel];		//latest one sample
		if(TEMP_OVERSAMPLING <= 1){
			return n/c;
		}else{
			return (n-(s*(c%TEMP_OVERSAMPLING)))/(c/TEMP_OVERSAMPLING);
			//^(the sum minus last incomplete 16) divided by oversample chunks
		}
	}else{
		return 0;
	}
}

void resetReading(uint8_t channel){
	xSemaphoreTake(tempMtxHandle, portMAX_DELAY);
	aggregate[channel] = 0;
	sampleCount[channel] = 0;
	xSemaphoreGive(tempMtxHandle);
}

int32_t getMilliCelcius(uint8_t channel){
	static uint32_t n, c;
	if(channel < TEMP_CHANNELS && sampleCount[channel]/TEMP_OVERSAMPLING){
		n = aggregate[channel];		//aggregated sum of samples
		c = sampleCount[channel];	//total number of samples aggreagated
		return adc_to_milliCelcius(n,c);
	}else{
		return 0;
	}
}

int32_t getMicroCelcius(uint8_t channel){
	static uint32_t n, c;
	if(channel < TEMP_CHANNELS && sampleCount[channel]/TEMP_OVERSAMPLING){
		n = aggregate[channel];		//aggregated sum of samples
		c = sampleCount[channel];	//total number of samples aggreagated
		return adc_to_microCelcius(n,c);
	}else{
		return 0;
	}
}

int32_t getMilliCelciusInternal(){
	static uint32_t n, c;
	n = aggregate[INTERNAL_CAHNNEL];		//aggregated sum of samples
	c = sampleCount[INTERNAL_CAHNNEL];	//total number of samples aggreagated
	return internalTemp_to_milliCelcius(n,c);
}

int32_t getMicroCelciusInternal(){
	static uint32_t n, c;
	n = aggregate[INTERNAL_CAHNNEL];		//aggregated sum of samples
	c = sampleCount[INTERNAL_CAHNNEL];	//total number of samples aggreagated
	return internalTemp_to_microCelcius(n,c);
}
