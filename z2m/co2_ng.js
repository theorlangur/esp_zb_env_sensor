const {battery, co2, temperature, humidity} = require('zigbee-herdsman-converters/lib/modernExtend');

const power_cfg_ext = {
    battery_read_voltage: () => {
        const res = battery({voltage:true});
        res.fromZigbee.push({
                cluster: 'genPowerCfg',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    if (msg.data.batteryPercentageRemaining !== undefined && msg.data['batteryPercentageRemaining'] < 255) {
                        const endpoint = meta.device.getEndpoint(1);
                        await endpoint.read('genPowerCfg', ['batteryVoltage']);
                    }
                    return {}
                }
            })
        return res;
    }
}

const definition = {
    zigbeeModel: ['Co2-NG'],
    model: 'Co2-NG',
    vendor: 'Orlangur',
    description: 'Automatically generated definition',
    extend: [power_cfg_ext.battery_read_voltage(), co2(), temperature(), humidity()],
    meta: {},
};

module.exports = definition;
