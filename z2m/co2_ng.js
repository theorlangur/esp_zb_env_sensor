const {battery, co2, temperature, humidity} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['Co2-NG'],
    model: 'Co2-NG',
    vendor: 'Orlangur',
    description: 'Automatically generated definition',
    extend: [battery({voltage:true, voltageReporting:true}), co2(), temperature(), humidity()],
    meta: {},
};

module.exports = definition;
