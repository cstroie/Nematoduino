// Word 0: Number of connected neurons (N_MAX)
// Word 1-(N_MAX): A address (offset from beginning) of each list of connections
// Word N_MAX+1: N_MAX plus the number of connections in the final neuron
// Word > (N_MAX+1): List of connections

// Connection words are formatted as follows:
// Bit 0-7: Lower part of neuron index
// Bit 8: High part of neuron index (9-bits reserved for index)
// Bit 9-15: Weight of connection

const uint16_t PROGMEM NeuralROM[] = {
  0x012c, 0x012e, 0x0141, 0x0152, 0x016f, 0x0183, 0x018b, 0x0199, 0x01ab, 0x01bb, 0x01bf, 0x01c3, 0x01d2, 0x01df, 0x01f0,
  0x0202, 0x0213, 0x0220, 0x0229, 0x0236, 0x023f, 0x0247, 0x0256, 0x0265, 0x026b, 0x0277, 0x027e, 0x0281, 0x0287, 0x029a,
  0x02a4, 0x02aa, 0x02b5, 0x02be, 0x02ca, 0x02d1, 0x02da, 0x02e4, 0x02ec, 0x02f2, 0x02f9, 0x0302, 0x030d, 0x0312, 0x0317,
  0x0327, 0x0338, 0x0342, 0x034a, 0x034f, 0x0353, 0x0359, 0x0362, 0x036a, 0x0373, 0x03a6, 0x03de, 0x0408, 0x0430, 0x0447,
  0x0463, 0x0478, 0x0490, 0x04a1, 0x04b0, 0x04c3, 0x04d5, 0x04e5, 0x04f6, 0x0502, 0x051e, 0x0531, 0x0542, 0x0553, 0x055c,
  0x0569, 0x0572, 0x057e, 0x0587, 0x058e, 0x0599, 0x05a1, 0x05a9, 0x05b4, 0x05c6, 0x05db, 0x05e9, 0x05f9, 0x0603, 0x0615,
  0x0622, 0x0631, 0x063d, 0x064c, 0x0653, 0x065b, 0x066b, 0x067f, 0x0690, 0x06a4, 0x06b2, 0x06bb, 0x06c4, 0x06d1, 0x06dd,
  0x06e8, 0x06f0, 0x06f9, 0x0702, 0x070c, 0x0731, 0x073c, 0x0750, 0x075d, 0x076c, 0x077e, 0x0798, 0x079d, 0x07a2, 0x07a5,
  0x07a8, 0x07ab, 0x07af, 0x07b4, 0x07bb, 0x07c8, 0x07d0, 0x07de, 0x07ef, 0x07f7, 0x0800, 0x0808, 0x0811, 0x081e, 0x082b,
  0x0836, 0x0840, 0x084a, 0x0854, 0x0858, 0x085e, 0x0865, 0x0871, 0x087e, 0x0887, 0x088d, 0x0895, 0x089b, 0x08a7, 0x08b1,
  0x08bb, 0x08cb, 0x08db, 0x08e3, 0x08ee, 0x08fb, 0x0907, 0x090d, 0x0912, 0x091a, 0x0922, 0x092c, 0x0935, 0x093b, 0x0946,
  0x094e, 0x0955, 0x0959, 0x0966, 0x0968, 0x096a, 0x0972, 0x0994, 0x09ba, 0x09c0, 0x09c7, 0x09cf, 0x09e4, 0x09fc, 0x0a10,
  0x0a26, 0x0a2d, 0x0a3e, 0x0a58, 0x0a68, 0x0a6d, 0x0a72, 0x0a81, 0x0a93, 0x0aab, 0x0ac3, 0x0ad2, 0x0adc, 0x0aeb, 0x0af4,
  0x0b02, 0x0b18, 0x0b28, 0x0b41, 0x0b55, 0x0b6e, 0x0b71, 0x0b74, 0x0b83, 0x0b9b, 0x0ba8, 0x0bba, 0x0bc5, 0x0bd2, 0x0be4,
  0x0bf1, 0x0c02, 0x0c18, 0x0c23, 0x0c29, 0x0c2d, 0x0c3a, 0x0c44, 0x0c47, 0x0c61, 0x0c76, 0x0c7c, 0x0c83, 0x0c89, 0x0c91,
  0x0c9b, 0x0ca1, 0x0ca5, 0x0ca8, 0x0cab, 0x0cb4, 0x0cc1, 0x0cc2, 0x0cc3, 0x0cc4, 0x0cc5, 0x0cc7, 0x0cca, 0x0ccf, 0x0cd3,
  0x0cdf, 0x0cec, 0x0cf7, 0x0d02, 0x0d14, 0x0d21, 0x0d31, 0x0d3e, 0x0d44, 0x0d4d, 0x0d55, 0x0d5e, 0x0d68, 0x0d76, 0x0d7f,
  0x0d8b, 0x0d93, 0x0d9b, 0x0da8, 0x0db3, 0x0dbb, 0x0dc1, 0x0dcb, 0x0de1, 0x0dec, 0x0dfb, 0x0e08, 0x0e13, 0x0e1d, 0x0e2b,
  0x0e3d, 0x0e4d, 0x0e62, 0x0e6e, 0x0e7a, 0x0e90, 0x0e9c, 0x0ea7, 0x0eb0, 0x0ebe, 0x0ec4, 0x0ed2, 0x0ee1, 0x0ef0, 0x0f00,
  0x0f16, 0x0f1d, 0x0f27, 0x0f28, 0x0f35, 0x0f3e, 0x0f45, 0x0f4d, 0x0f59, 0x0f63, 0x0f6c, 0x0f74, 0x0f7c, 0x0f87, 0x0f8e,
  0x0f92, 0x0f98, 0x2b01, 0x3807, 0x0d02, 0x3b01, 0x3a02, 0x7001, 0x4305, 0x0102, 0x3602, 0xf502, 0xbe01, 0xb301, 0x0401,
  0xbd01, 0xc503, 0x3901, 0xc701, 0x0c01, 0x3704, 0x3805, 0x0d01, 0xc605, 0x3b01, 0xcc01, 0xbd01, 0x0501, 0xc801, 0x3501,
  0xf402, 0x0001, 0x4203, 0x3902, 0xb401, 0x0c01, 0x2c01, 0x3701, 0xeb01, 0x9501, 0xcb01, 0x4501, 0xcf02, 0xef01, 0x7d01,
  0xcc01, 0xdb01, 0x0301, 0x3603, 0x3181, 0x3502, 0x0001, 0xc402, 0xc001, 0x5201, 0xfa01, 0x1001, 0xe701, 0x6f01, 0x3b01,
  0xc205, 0xd901, 0x8301, 0x4601, 0x5001, 0xc303, 0xb901, 0xb501, 0xc401, 0xc207, 0x3a02, 0x7001, 0x4301, 0x0101, 0x3601,
  0x1601, 0xe001, 0x3c01, 0x3505, 0x4402, 0x6f01, 0xd002, 0x0202, 0x9602, 0x4501, 0xc304, 0x5301, 0x3305, 0x140c, 0xc201,
  0x9901, 0xc902, 0xf002, 0x0002, 0xb90f, 0xba10, 0x1301, 0xc903, 0x0102, 0x0b01, 0x4b01, 0xef01, 0xb201, 0xf102, 0xfd01,
  0xc303, 0x1508, 0x2c01, 0x3404, 0x2b02, 0x0d01, 0x3a01, 0x2803, 0xd901, 0x3603, 0x4a02, 0x1602, 0x3502, 0x0a06, 0x4303,
  0x4201, 0x9902, 0x3901, 0xc701, 0x0c07, 0x0701, 0x3702, 0x3802, 0x0d0a, 0x0601, 0x4d03, 0x3a02, 0x2801, 0x0b0a, 0x3602,
  0xaa01, 0xbe01, 0xbd01, 0x4301, 0x3905, 0x9601, 0x2c03, 0x3701, 0x0c01, 0x1207, 0x1101, 0x0901, 0x0801, 0x0d01, 0x130d,
  0x2801, 0x2b01, 0x4d01, 0x2d02, 0x1401, 0x7101, 0x2803, 0xd901, 0x0b01, 0x2901, 0x0e02, 0x0001, 0xc001, 0x3103, 0x0c0a,
  0x4801, 0x4902, 0x0d0e, 0xc102, 0x2801, 0x0101, 0x4d03, 0x0501, 0x2a01, 0x0a01, 0x2e02, 0x1501, 0x0702, 0x4c01, 0xb601,
  0xc601, 0x0801, 0xde02, 0xbc04, 0x6e01, 0xc502, 0x3502, 0xf304, 0xdd02, 0x6f01, 0x2801, 0xc303, 0xc001, 0xcb01, 0x1201,
  0x3705, 0xc601, 0x3803, 0xc203, 0xb601, 0x3b01, 0xcc01, 0x3601, 0xca01, 0x6e02, 0xf203, 0xc501, 0x0e83, 0xf401, 0xdd01,
  0xbb04, 0x0901, 0x5f01, 0xb901, 0x3802, 0x3d04, 0x3a01, 0x1701, 0xf001, 0xeb01, 0x4201, 0x2902, 0x4002, 0xb301, 0x4101,
  0x0a05, 0x3901, 0x3c01, 0x3102, 0xc001, 0x3e01, 0x9801, 0xb001, 0x3d01, 0x7101, 0x3a01, 0x3203, 0x0b05, 0xc101, 0x2a02,
  0xda01, 0x3002, 0x7202, 0x3e01, 0xbc02, 0x1102, 0x0905, 0x2a02, 0xbb01, 0x2703, 0x0201, 0x4e03, 0x3402, 0x4f03, 0x3301,
  0x2801, 0x0804, 0x0901, 0x2901, 0xbb02, 0x0a02, 0x2701, 0x1002, 0xbf01, 0x0c02, 0x3401, 0x1301, 0x140d, 0x4d01, 0xbb04,
  0x4c01, 0xc501, 0x7201, 0xb907, 0x4803, 0x4901, 0xba06, 0x7101, 0xc601, 0xbc02, 0x0501, 0x1508, 0x1201, 0x2b01, 0x0d08,
  0xc201, 0x1181, 0xb908, 0x6c01, 0x2901, 0xf007, 0xc504, 0xee09, 0x0a03, 0x2701, 0x3c05, 0x1502, 0x0c02, 0x4901, 0x0d01,
  0x1402, 0xc604, 0x3b04, 0x0b01, 0x6c01, 0xba07, 0xef05, 0x2a01, 0xf301, 0xf103, 0x3c01, 0x0c08, 0x2c01, 0x3b02, 0xd001,
  0x3c01, 0x3501, 0x0201, 0xbf01, 0x5203, 0xb501, 0xab01, 0x3b01, 0x5006, 0xd901, 0x3a01, 0xce01, 0x4701, 0x5402, 0xe401,
  0xaa02, 0xab03, 0x5501, 0xe601, 0x5105, 0x4701, 0xcd01, 0x5301, 0xf401, 0xef01, 0xdf03, 0x0301, 0xe002, 0xee02, 0xf401,
  0xf301, 0xdc01, 0x4f02, 0x3804, 0xba01, 0xab02, 0xc202, 0x3a01, 0x4501, 0x3603, 0xb107, 0xb209, 0x3501, 0x4402, 0xfc01,
  0x4201, 0xc301, 0x3901, 0xb903, 0x4e02, 0x3701, 0x3483, 0x1f85, 0x2481, 0x3602, 0x3183, 0x3503, 0x4983, 0x5602, 0x0781,
  0x4c84, 0x4082, 0x3f83, 0x5882, 0x3601, 0x3501, 0x5783, 0x4281, 0x4381, 0x4181, 0x2382, 0x4481, 0x9c02, 0x9b01, 0x5c81,
  0x5981, 0x5a81, 0x5b81, 0x3482, 0x248a, 0x6601, 0x3383, 0x4b83, 0x0882, 0x5701, 0x5f01, 0x4c83, 0x3583, 0x258f, 0x2481,
  0x6601, 0x3601, 0x3683, 0x3502, 0x4e83, 0x5701, 0x5801, 0x0982, 0x4d83, 0x268b, 0x2201, 0x5801, 0x3882, 0x3782, 0x4f83,
  0x5082, 0x2789, 0x6701, 0x3a83, 0x3601, 0x3782, 0x5283, 0x3501, 0x4f82, 0x0b81, 0x3801, 0x5282, 0x3983, 0x3a82, 0x3601,
  0x0c81, 0x288d, 0x5a02, 0x3501, 0x5183, 0x3802, 0x3c83, 0x3605, 0x3982, 0x3506, 0x5483, 0x5182, 0x3702, 0x3e83, 0x3b82,
  0x3603, 0x5382, 0x3504, 0x5683, 0x4083, 0x3d82, 0x3602, 0x3504, 0x6d07, 0x5582, 0x5883, 0x0d02, 0x1306, 0x4d01, 0xba01,
  0x0501, 0x0a03, 0x0c07, 0x120d, 0x4c04, 0x4901, 0x0d0a, 0x130e, 0x0801, 0x0b03, 0x4d01, 0x0a01, 0x0902, 0x0c02, 0x1202,
  0x4c01, 0x1401, 0x0c03, 0x0a09, 0x1102, 0x3101, 0x1001, 0x0d02, 0x0b0a, 0x1501, 0x1301, 0x1401, 0x3a02, 0xb904, 0xd901,
  0xbd02, 0xc701, 0x0403, 0x3502, 0x0002, 0x0a07, 0x3902, 0xc501, 0x3101, 0x0c05, 0x2c01, 0x3706, 0x2b01, 0x3803, 0x0d03,
  0x3a01, 0x3201, 0xbe02, 0x0103, 0x0b0a, 0x3605, 0xba02, 0x0502, 0xb201, 0xda01, 0x3c03, 0x3905, 0x1501, 0x7201, 0x1401,
  0x2801, 0x4d01, 0xbb01, 0x1202, 0x0a02, 0x3102, 0x0c01, 0x2e01, 0x4c01, 0x4d01, 0x0d01, 0x2d01, 0x0b02, 0x0a01, 0x2702,
  0x2c01, 0x4c01, 0x3001, 0x3104, 0x7201, 0x7101, 0xb30e, 0x7201, 0xb40d, 0x2f01, 0x3204, 0xb305, 0x3201, 0xd901, 0x0e02,
  0x0a0b, 0x0c02, 0x4901, 0xc101, 0x5501, 0x0b0b, 0x0f01, 0xda01, 0x3101, 0x2c01, 0xb404, 0x1101, 0x3b03, 0x3a01, 0x3503,
  0x4a01, 0xbb09, 0xb905, 0x3401, 0x3301, 0xba06, 0x1301, 0x4b01, 0xbc0d, 0x3601, 0x3c04, 0xfd01, 0x1001, 0x6f01, 0x4202,
  0xab0b, 0xaa0c, 0x6302, 0x0487, 0x0582, 0x6404, 0xc603, 0x0d88, 0x0386, 0x3607, 0xb101, 0xe301, 0x0c93, 0x0181, 0x1e04,
  0x1d03, 0x0685, 0x0783, 0x0283, 0x0a8a, 0x0b82, 0xe104, 0x0988, 0x2003, 0x1f01, 0x1c03, 0x240e, 0x2301, 0x2204, 0x2101,
  0x260c, 0x2509, 0x590a, 0x5a08, 0x5b15, 0x5c04, 0x5604, 0x5704, 0x5806, 0x5d04, 0x5e03, 0x3801, 0xe501, 0x4001, 0xfe01,
  0x0883, 0x3901, 0x1885, 0x8702, 0x9d01, 0xab08, 0x0682, 0x6101, 0xaa07, 0x6303, 0x9e01, 0x0581, 0x6405, 0x048f, 0x0c90,
  0xc601, 0x0301, 0x0385, 0x0081, 0xe301, 0xe206, 0x3506, 0x1e06, 0x1d02, 0x3c02, 0x0787, 0x0a85, 0x0b84, 0xe101, 0x0984,
  0x2002, 0x1f02, 0x1c03, 0x2408, 0x2303, 0x2202, 0x2101, 0x2606, 0x2509, 0xff01, 0x5908, 0x5a07, 0x5b0d, 0x5c03, 0x5608,
  0x5704, 0x5805, 0xc502, 0x5d09, 0x5e02, 0x2382, 0x8803, 0x3b02, 0xc201, 0x3a02, 0x0885, 0x3901, 0x188a, 0x8701, 0x3701,
  0x6c01, 0x6101, 0x6301, 0x6201, 0x6502, 0x6402, 0x3b01, 0x1b01, 0x0381, 0xbc01, 0x3607, 0x3507, 0x1b81, 0xbb01, 0x1d01,
  0x3c02, 0xbf01, 0x0b81, 0x2001, 0x2402, 0x2301, 0x2201, 0x2101, 0x2601, 0x0681, 0x3804, 0xb001, 0xe501, 0x1082, 0x3a02,
  0xec01, 0x0f82, 0x4601, 0x1184, 0x0e81, 0x1682, 0x1581, 0x1481, 0x1381, 0x3901, 0x1881, 0x1787, 0x6c01, 0x5f03, 0x6101,
  0x7201, 0x6301, 0x6201, 0x6501, 0x6401, 0x6001, 0x1081, 0x0f81, 0x6601, 0xbc01, 0x3607, 0x0c81, 0x3506, 0x0d82, 0xbb01,
  0x1d01, 0xbf02, 0x0881, 0x2001, 0x1c01, 0x2403, 0x2302, 0x2201, 0x2101, 0x5a01, 0x2081, 0x2581, 0xec01, 0x1281, 0x1181,
  0x1682, 0x1582, 0x1381, 0x1886, 0x1783, 0xaf02, 0x3704, 0x6f01, 0xaa01, 0x0102, 0x3613, 0xe301, 0xe201, 0x350d, 0x1e02,
  0x1d01, 0xe101, 0x0981, 0x1c01, 0x2201, 0x2101, 0x7001, 0x5901, 0x5a01, 0x5601, 0x5701, 0x5804, 0x5d01, 0x4702, 0x8701,
  0x0601, 0x4202, 0x6e01, 0x6201, 0x0481, 0x360f, 0xe301, 0xe203, 0x3510, 0x0002, 0x1d01, 0x0681, 0x0782, 0x0a81, 0xe101,
  0x2201, 0x7001, 0x5901, 0x5a02, 0x5602, 0x5701, 0x5801, 0x5d01, 0x5e01, 0xa901, 0x3902, 0x8702, 0x3701, 0x1c01, 0x2481,
  0xab01, 0xc603, 0xe207, 0xd201, 0x3607, 0x2581, 0xd601, 0xe303, 0x5901, 0xc502, 0x350c, 0x5605, 0x5701, 0x5803, 0x3c01,
  0x0783, 0x0285, 0xe106, 0xb601, 0xd201, 0xd101, 0x6101, 0x3b01, 0x3610, 0xe303, 0xe203, 0x3507, 0x0681, 0x0782, 0x0281,
  0xe102, 0x0981, 0x2001, 0x1f02, 0x1c03, 0xc602, 0xd601, 0x5605, 0x5703, 0x5801, 0xc503, 0x3a01, 0x0881, 0x3802, 0x7101,
  0x6881, 0x6781, 0x4301, 0xb402, 0xb301, 0x4004, 0x9e01, 0x4601, 0x0e81, 0x4107, 0x4201, 0x3f01, 0xaf02, 0x3e1e, 0x3701,
  0x3805, 0x3d18, 0x2f01, 0x8182, 0x4301, 0x4004, 0xb301, 0x4102, 0x1c81, 0x6a82, 0x4201, 0x3101, 0x7201, 0x2181, 0x3701,
  0x3802, 0xb001, 0x3d01, 0xb201, 0xb601, 0x3b01, 0x3a01, 0xb401, 0x3603, 0xc101, 0x4601, 0x9f02, 0x4201, 0x3c01, 0xc001,
  0xaa01, 0x5d01, 0x0481, 0x3701, 0x1f81, 0x3801, 0x3d02, 0xf101, 0xc601, 0xb301, 0xa201, 0xc903, 0x4b01, 0xef01, 0x0503,
  0xb202, 0x4102, 0x4201, 0x3901, 0xb402, 0x3e05, 0x3701, 0x3801, 0x0601, 0xc201, 0x3a01, 0x4304, 0x1b02, 0xc904, 0x4002,
  0xb103, 0x3e02, 0xf001, 0x3d01, 0xee01, 0xaf01, 0x0702, 0x3702, 0x3804, 0xb001, 0xaa02, 0xab05, 0x3b01, 0x3a02, 0x4304,
  0xc101, 0x3601, 0x4002, 0xa602, 0x3502, 0xca02, 0x3901, 0x7201, 0x3e01, 0x3701, 0x3801, 0xab04, 0x3a03, 0xb401, 0x3601,
  0xe201, 0x3501, 0x4205, 0x3c03, 0x3901, 0xaa03, 0x3703, 0x9d03, 0x4502, 0x6c01, 0xef01, 0xee01, 0x9e01, 0xb602, 0x0301,
  0x1b02, 0x0e84, 0xb101, 0xb201, 0xd801, 0x3c01, 0xae01, 0xc601, 0xf301, 0xc502, 0x3b02, 0xc201, 0xde01, 0x0f81, 0xbe01,
  0x4701, 0xbd01, 0xf101, 0xe901, 0x3701, 0xb301, 0xef02, 0xf302, 0xc602, 0x6681, 0xd001, 0x1b01, 0xd701, 0xb106, 0xc301,
  0x5001, 0xbd01, 0xee02, 0x4402, 0xf101, 0x0201, 0xdd01, 0xc502, 0xf201, 0xe204, 0x7dfb, 0xe303, 0x3b01, 0xb701, 0x6601,
  0x6b02, 0x6e09, 0x6d01, 0xb201, 0x66fb, 0x5701, 0x7201, 0xae01, 0x3e01, 0xe105, 0x2284, 0x3806, 0xb503, 0xab05, 0x0301,
  0x1701, 0x4301, 0x5102, 0x5003, 0x5601, 0x1801, 0xec01, 0x0282, 0x3902, 0xbf01, 0xaa04, 0xaf01, 0x3706, 0x4901, 0x140a,
  0x0805, 0x2901, 0x4a01, 0x0001, 0x0a01, 0x2704, 0x1201, 0x1302, 0x0b01, 0xc102, 0x2802, 0x2701, 0x4b01, 0x0503, 0xc902,
  0x0907, 0xc301, 0x1501, 0x4801, 0x3401, 0x3301, 0x0d01, 0x1409, 0xd901, 0x4b01, 0x0409, 0xee01, 0xb903, 0x3701, 0x3802,
  0xba01, 0xc902, 0xbd01, 0x0504, 0x4a01, 0x2a01, 0xda01, 0xf101, 0x1504, 0x2c02, 0x3401, 0x0d01, 0x4d01, 0xb903, 0x0b04,
  0x3501, 0x0a02, 0x2701, 0x0c01, 0x120a, 0x0d04, 0x1309, 0x0b01, 0x2a01, 0x4c05, 0x2701, 0x1204, 0x4f02, 0xba05, 0xc201,
  0x3b01, 0xc901, 0xbc07, 0x3601, 0xc301, 0xbb01, 0x3c04, 0x0c01, 0xc201, 0x3b02, 0xc901, 0x3501, 0xbb04, 0xb905, 0x1201,
  0x4e01, 0xb002, 0x7101, 0x4301, 0xf601, 0x4001, 0xdd01, 0x0203, 0xaf02, 0xb001, 0xaa01, 0x0301, 0x4001, 0x3503, 0xf701,
  0x1801, 0x4202, 0xe401, 0x7204, 0xaf02, 0x7b04, 0xfe02, 0xef01, 0x9502, 0xfa04, 0xd904, 0xf602, 0xbe02, 0xca01, 0xd103,
  0xc702, 0xbd01, 0xe701, 0xbb02, 0xc401, 0x3c05, 0xdc04, 0x9701, 0x9802, 0xdb04, 0xef01, 0x3b06, 0xd202, 0x7e01, 0xbc01,
  0xbe03, 0xca01, 0xe601, 0x5101, 0xbd04, 0xda01, 0xf701, 0xc401, 0xcd01, 0xfb02, 0x9608, 0xdc01, 0xff01, 0x7c05, 0x0601,
  0x9504, 0xe801, 0x9901, 0xbe04, 0xc701, 0x5f81, 0xbd07, 0xdb01, 0xcd04, 0xc401, 0x7f02, 0x3c03, 0xf802, 0xf901, 0xce02,
  0x3b05, 0x7781, 0x8001, 0xbe02, 0x8602, 0xc801, 0xbd02, 0x2a01, 0xc401, 0x9a01, 0xe902, 0x9607, 0xcb01, 0xdc02, 0x3488,
  0x1f91, 0x2481, 0x6604, 0x3606, 0xe303, 0x5901, 0xe202, 0x3502, 0x4c88, 0x2001, 0x1f02, 0x2585, 0x0282, 0x3582, 0x6601,
  0x3602, 0xe201, 0x3382, 0x4b82, 0x3502, 0x3481, 0x4e82, 0x248b, 0x3682, 0x1f82, 0x4c82, 0x4d82, 0x3585, 0x2599, 0x2686,
  0x2102, 0x6701, 0x3602, 0x3885, 0x3685, 0x5902, 0x4e85, 0x5085, 0x6101, 0x4d85, 0x278f, 0x268c, 0x1581, 0x6701, 0x3a85,
  0x3602, 0x3884, 0x3784, 0x5285, 0x3503, 0x4f84, 0x5601, 0x5801, 0x6102, 0x5084, 0x2781, 0x2302, 0x2890, 0x3605, 0x3a84,
  0x3985, 0x5284, 0x3501, 0x5185, 0x6201, 0x0881, 0x0982, 0x2783, 0x2684, 0x2883, 0x3602, 0x3a84, 0x3984, 0x3884, 0x3786,
  0x5284, 0x350a, 0x5484, 0x4f84, 0x5184, 0x3c84, 0x5084, 0x3e84, 0x3d84, 0x3b84, 0x5584, 0x5384, 0x3502, 0x5684, 0x4084,
  0x3f84, 0x3d84, 0x5884, 0x3601, 0x5584, 0x5784, 0x5e01, 0x4284, 0x4384, 0x4184, 0x2381, 0x4484, 0x3f84, 0x5784, 0x6b01,
  0x9b01, 0xa301, 0x5c84, 0xbf01, 0x5d01, 0x5984, 0x5a84, 0x5b84, 0x2001, 0x1f01, 0x0d01, 0x248f, 0x1f95, 0x3803, 0x660a,
  0x6c01, 0x1c01, 0x3381, 0x1281, 0x4b81, 0x3481, 0xca01, 0x1381, 0x2581, 0xbf01, 0x6001, 0x6201, 0x4c81, 0x3801, 0x2597,
  0x2781, 0x268e, 0x3583, 0x6703, 0x5083, 0x3883, 0x3783, 0x3683, 0x0e82, 0x4f83, 0x4e83, 0x5805, 0x5f01, 0x6106, 0x4d83,
  0x3801, 0x279a, 0x2689, 0x2201, 0x2101, 0x680a, 0x6704, 0x3a83, 0x3984, 0x3883, 0x3783, 0x2887, 0x5901, 0x5283, 0x4f83,
  0x5184, 0x5083, 0x6006, 0x6201, 0x3701, 0x3801, 0x1181, 0x6803, 0x3c82, 0x3a82, 0x3982, 0x288d, 0x5282, 0x5482, 0x5182,
  0x1381, 0x5f01, 0x6101, 0x3701, 0x3801, 0x3e82, 0x3d82, 0x3b82, 0x3602, 0x5382, 0x5582, 0x5682, 0x3701, 0x3801, 0x4082,
  0x3f82, 0x3d82, 0x5882, 0x5582, 0x3503, 0x5782, 0x3702, 0x4282, 0x3801, 0x4182, 0x4482, 0x3f82, 0x4382, 0x2382, 0x5c82,
  0x5b82, 0x5782, 0x5982, 0x5a82, 0x3702, 0x34fa, 0x3801, 0x2482, 0x35f9, 0x6703, 0x33fa, 0x4bfa, 0x4efa, 0x36fa, 0x1f84,
  0x4cfa, 0x4df9, 0x35fa, 0x2581, 0x2683, 0x6802, 0x6601, 0x50fa, 0x38fa, 0x37f9, 0x4ff9, 0x5801, 0x4dfa, 0x6702, 0x3af9,
  0x39f7, 0x37f9, 0x6901, 0x52f9, 0x4ff9, 0x51f7, 0x6801, 0x3cf9, 0x3bf9, 0x39f9, 0x53f9, 0x54f9, 0x51f9, 0x2a81, 0x1b81,
  0x40f9, 0x3ef9, 0x3df9, 0x2081, 0x2b81, 0x55f9, 0x1781, 0x58f9, 0x56f9, 0x42f9, 0x43f9, 0x41f9, 0x3ff9, 0x44f9, 0x5cf9,
  0x57f9, 0x59f9, 0x5af9, 0x5bf9, 0x9d03, 0xab01, 0xc903, 0xef02, 0xee03, 0x5f01, 0x6102, 0x6001, 0x6301, 0x6201, 0x6501,
  0x6402, 0xaa01, 0x3b09, 0x1b04, 0x3601, 0x3503, 0x0581, 0x3c05, 0xe101, 0x3301, 0x1403, 0xc601, 0xdf01, 0xe001, 0x0681,
  0xb502, 0x3401, 0x3801, 0xba01, 0x1082, 0xde01, 0xf003, 0x0e81, 0xf102, 0xb901, 0x3702, 0xa301, 0x2607, 0x6b03, 0x6e03,
  0x4601, 0x0d81, 0xb101, 0x2cfb, 0x1881, 0x5d02, 0x9b01, 0x1f85, 0xb20d, 0x0d05, 0xb601, 0xc205, 0x0d81, 0x4501, 0xbc01,
  0x3607, 0xd702, 0xb102, 0x4609, 0x3505, 0xd804, 0x4402, 0xbb01, 0xc305, 0x0c02, 0x2084, 0x3701, 0x3805, 0x0d02, 0x0302,
  0x3a0d, 0x7003, 0x6c01, 0x3611, 0x350f, 0xc401, 0x0202, 0x3907, 0x0c01, 0x3704, 0x3801, 0x0d01, 0x3b04, 0x0301, 0x3a02,
  0x4301, 0x3605, 0x0e81, 0x350c, 0x6f04, 0x6c01, 0x3c02, 0x3901, 0xaa02, 0x3705, 0x2b01, 0xe202, 0x3d06, 0x1402, 0x3a02,
  0x4b02, 0xc502, 0x4a01, 0x8c87, 0x0a01, 0x4201, 0x3001, 0x3101, 0x1501, 0x7201, 0xc003, 0x2c02, 0x1d83, 0x0d01, 0x7102,
  0x0c01, 0xc104, 0xe301, 0x1a83, 0x1b81, 0x0a81, 0xe101, 0x1401, 0x2201, 0x5a01, 0x5b01, 0x8c86, 0x4201, 0x1501, 0x2b02,
  0xb002, 0x3d01, 0x2682, 0x3a01, 0x5101, 0x4601, 0x4a01, 0xda01, 0xb401, 0xc801, 0x7701, 0xc701, 0x7901, 0x7401, 0xc801,
  0x7301, 0x7701, 0x7901, 0xc701, 0x7301, 0x8904, 0x7401, 0x7301, 0x8904, 0x7401, 0x8a02, 0x8904, 0x8b02, 0x7902, 0x7605,
  0x7505, 0x8904, 0x8f02, 0x7304, 0x8902, 0x9204, 0x7403, 0x9402, 0x7602, 0x8f02, 0x8e01, 0x7502, 0x9302, 0x7701, 0xb501,
  0x9501, 0xce01, 0x2d81, 0x2e81, 0x3082, 0xd601, 0xc702, 0xd104, 0xfe01, 0x7d01, 0xc401, 0x7c01, 0x7b01, 0x7e01, 0xd205,
  0xd601, 0xc805, 0x4683, 0x4584, 0x9601, 0x7b02, 0xd003, 0x2d83, 0xd501, 0x2f83, 0xcd05, 0x3184, 0x5f83, 0xd104, 0x5d83,
  0x7f01, 0x3c02, 0xd202, 0xcf01, 0xdb01, 0xce02, 0x3b01, 0xfd02, 0x8001, 0xcf04, 0xd204, 0x7483, 0x7683, 0xcd03, 0xd101,
  0xd402, 0x4783, 0x4583, 0x3c01, 0xd002, 0x7c02, 0x8001, 0x0081, 0xc704, 0x5e84, 0x5d85, 0x7d02, 0xcd05, 0xd301, 0xce0a,
  0x7e02, 0x7485, 0x7585, 0xc806, 0x8401, 0x8601, 0x7f01, 0xd501, 0x3301, 0x7b07, 0xd503, 0xf603, 0xc70a, 0xd404, 0xbb01,
  0x9702, 0xd502, 0xbe01, 0xd601, 0xc80b, 0xd402, 0xf703, 0x5301, 0xd301, 0x7c07, 0x3b01, 0x9908, 0xd502, 0xd602, 0xbd01,
  0x7d01, 0xfc02, 0xc407, 0xcf03, 0x0202, 0xd001, 0xd901, 0x9705, 0x9802, 0xfb01, 0x0301, 0x8001, 0x7e01, 0xcf01, 0xd601,
  0xd402, 0xc406, 0x9a07, 0xfd01, 0xda01, 0x9601, 0x4f01, 0xf803, 0x9901, 0xd504, 0x8301, 0xd601, 0xc701, 0xd401, 0xc402,
  0x7f07, 0xb901, 0xba02, 0x8006, 0xd502, 0xfd01, 0xd603, 0xc80f, 0xd403, 0xc403, 0x9a01, 0xf904, 0xa501, 0x3a02, 0xb701,
  0x3606, 0x3506, 0xb501, 0xa101, 0x4201, 0x3904, 0xaf01, 0xb502, 0xab03, 0x3a03, 0x4301, 0xb701, 0x3607, 0xa901, 0x3503,
  0x3901, 0xa601, 0x7602, 0x7502, 0x7701, 0x7801, 0x7703, 0x9204, 0x8b01, 0x7403, 0x8f01, 0x7303, 0x7703, 0x9204, 0x8d01,
  0x7403, 0x8f01, 0x7303, 0x8c01, 0x8d01, 0x9403, 0x9202, 0x7903, 0x7802, 0x7a01, 0x7304, 0x9001, 0x8902, 0x9302, 0x7404,
  0x9101, 0x9001, 0x9403, 0x9202, 0x7702, 0x7903, 0x7806, 0x7a01, 0x7304, 0x8c01, 0x8902, 0x9302, 0x7404, 0x9101, 0x8f01,
  0x9401, 0x7701, 0x790d, 0x8b01, 0x7a03, 0x8a01, 0x8e06, 0x9301, 0x8f04, 0x7901, 0x8b02, 0x7a01, 0x8a02, 0x8902, 0x7601,
  0x8b02, 0x7403, 0x8a02, 0x7303, 0x8902, 0x7501, 0x7701, 0x7701, 0x8b02, 0x7403, 0x8a02, 0x7303, 0x8902, 0x8d01, 0x7701,
  0x7902, 0x7801, 0x8b02, 0x7401, 0x8a02, 0x7301, 0x8c01, 0x8901, 0x9102, 0x9002, 0x8d02, 0x7606, 0x7902, 0x7803, 0x7402,
  0x7301, 0x8c02, 0x7506, 0x7a02, 0x7702, 0x8d02, 0x7606, 0x7902, 0x7803, 0x7402, 0x7302, 0x8c02, 0x7506, 0x7a02, 0x7702,
  0x5203, 0xfe01, 0xc201, 0x7b01, 0x5404, 0xf203, 0xf504, 0xcd07, 0xd101, 0xf304, 0xd402, 0xbb08, 0x7f02, 0x3c15, 0x9602,
  0xcf02, 0xf404, 0x9502, 0x3b10, 0x5506, 0xd502, 0xbc0a, 0xd203, 0xf503, 0xce0a, 0x8401, 0xf301, 0x8001, 0xc301, 0x5301,
  0xcf03, 0x7c03, 0x5201, 0xc201, 0xce04, 0xfa01, 0xec03, 0xbe01, 0xd101, 0xbb02, 0xd201, 0xbc02, 0xbe01, 0xcd03, 0xbd01,
  0xc401, 0xed02, 0xc301, 0xfb01, 0xdc01, 0x5302, 0x0601, 0xc201, 0xfa01, 0x5401, 0xd204, 0xc701, 0x8501, 0xbd01, 0xbb01,
  0xc401, 0x7f01, 0xcd01, 0xea03, 0xeb04, 0xce01, 0x5501, 0xd501, 0xbc01, 0xbe01, 0xc802, 0xd104, 0xc402, 0x8001, 0xc301,
  0xfb01, 0xb001, 0x4182, 0x6b01, 0x1e01, 0x5e01, 0x2383, 0x7281, 0x2382, 0xbf02, 0x1e02, 0x8881, 0xb502, 0xab01, 0x2181,
  0x6c18, 0x0d81, 0x4406, 0xae01, 0x9e03, 0x9d03, 0xab01, 0x2b81, 0x6c23, 0x0c81, 0x4410, 0xaa01, 0xae01, 0xb302, 0x3d03,
  0x3a01, 0xa205, 0x6c02, 0x4001, 0xa002, 0x4101, 0xa105, 0x3f05, 0xb302, 0x4101, 0xa205, 0x6c01, 0xb103, 0x9f02, 0xa101,
  0x3f03, 0x5d01, 0xa203, 0x3606, 0x3509, 0x3901, 0xaa0d, 0x0581, 0x3d01, 0xab03, 0x3a01, 0x3607, 0x4001, 0x3507, 0xa103,
  0x3901, 0xaa06, 0x5d01, 0x0582, 0xa501, 0x6c06, 0x3501, 0x0583, 0xa401, 0xaa02, 0x5e01, 0x8701, 0x8801, 0xab09, 0xa302,
  0x6c08, 0x4101, 0x5e02, 0x0582, 0xa301, 0xaa01, 0x7101, 0x8701, 0x8801, 0x9d02, 0x2301, 0x3a04, 0x6c05, 0x3601, 0xaa02,
  0x3504, 0xb502, 0x3901, 0x7201, 0x9e03, 0xab01, 0xdd05, 0xf006, 0xde04, 0xf106, 0x8801, 0x3a06, 0x360b, 0xb104, 0x3508,
  0x3907, 0x3f01, 0xaf01, 0xab07, 0x6c05, 0x6104, 0x6003, 0x6302, 0x6203, 0x6503, 0x6402, 0x3b03, 0x0f83, 0x3604, 0xca02,
  0x3503, 0x3c01, 0xbf05, 0x1c01, 0x4302, 0x5a01, 0x5b01, 0x5701, 0x4204, 0x380c, 0xa501, 0x1081, 0x3a02, 0xec02, 0x1281,
  0x1582, 0x1481, 0x1381, 0x3905, 0x1882, 0x1781, 0x3705, 0x9d02, 0xa401, 0x6103, 0x6001, 0x6301, 0x6204, 0x6501, 0x6402,
  0xaa08, 0x3b01, 0x1b01, 0x360a, 0x0c82, 0x350c, 0x0d81, 0x3c01, 0xbf05, 0x1f01, 0x1383, 0x0f81, 0xb501, 0x6f01, 0x4203,
  0xb802, 0x5e01, 0x8801, 0x3806, 0x3a01, 0xac01, 0xb702, 0x4601, 0x1683, 0x1582, 0xed02, 0x3905, 0x1781, 0xa601, 0x3708,
  0xab06, 0x3606, 0x6a01, 0x3506, 0xaa01, 0x2086, 0xab0a, 0x3609, 0xac01, 0x6c03, 0x3506, 0x0d81, 0xaa0d, 0x9d01, 0x6c03,
  0x4601, 0x4701, 0xb501, 0x440b, 0xaa02, 0x9e01, 0x3803, 0xb005, 0x6583, 0xb601, 0x3b01, 0x3a03, 0x4305, 0x6602, 0xb701,
  0xaa01, 0xa901, 0x5102, 0x4602, 0x5001, 0x3502, 0x4205, 0x3903, 0x3f01, 0xc001, 0x3e01, 0xb401, 0x2881, 0xaa01, 0x7202,
  0xb602, 0x3b03, 0x6801, 0x6982, 0x1a81, 0x1b81, 0x4301, 0x4204, 0x2281, 0x3802, 0x3502, 0x2981, 0x3a01, 0xb702, 0x5102,
  0x4602, 0x5001, 0xa902, 0x6881, 0xaf01, 0x3701, 0x3806, 0x2581, 0xab03, 0xa904, 0xc202, 0x1b08, 0x4506, 0x3a02, 0x3601,
  0xa003, 0x6e02, 0xb201, 0x3502, 0x4101, 0x0001, 0xb601, 0x3c01, 0x4401, 0x2382, 0x3705, 0xab07, 0x6e0e, 0x4401, 0xaa04,
  0x1b0b, 0x6701, 0x3602, 0xb101, 0x3501, 0xc601, 0x3805, 0xba02, 0x2781, 0x2681, 0x4003, 0x0501, 0x4604, 0xda01, 0xc301,
  0x3704, 0x2c01, 0xb401, 0x7102, 0x2f01, 0xd901, 0x0001, 0x0a03, 0x3105, 0xb402, 0x1f81, 0x4902, 0x3d01, 0xb601, 0x3208,
  0x2801, 0x0101, 0x0b07, 0xb301, 0xc101, 0x6e01, 0x4601, 0x6601, 0x7201, 0xaf01, 0x3e01, 0x3701, 0x7b01, 0x9d01, 0xab01,
  0x4203, 0x6c03, 0xc703, 0xc803, 0x4401, 0x6101, 0x6001, 0x9e01, 0x7c01, 0x0001, 0x4302, 0xe101, 0xf601, 0x2301, 0x8001,
  0x7f01, 0x5e01, 0x8801, 0x3804, 0x1701, 0xa602, 0x8701, 0x3704, 0xd601, 0x0d05, 0xc202, 0x4507, 0xbc01, 0xd702, 0x6e02,
  0xef01, 0x4602, 0xd803, 0x4409, 0xb101, 0xc401, 0xc303, 0x0c03, 0xbb01, 0xb801, 0xb602, 0xab02, 0x0581, 0x4201, 0xb601,
  0x3601, 0xab02, 0x0581, 0x3a01, 0xba01, 0xd006, 0xce07, 0xcb02, 0x5401, 0xd20b, 0xf208, 0xf50b, 0xd109, 0xf30a, 0xf406,
  0xcc04, 0xcd0c, 0xcf06, 0xe602, 0xf507, 0xd008, 0xe801, 0xce0b, 0x5501, 0x7e01, 0xcf03, 0xd20a, 0xe701, 0xf207, 0xd10c,
  0xf307, 0xf40d, 0xcb01, 0xcd0a, 0xb904, 0xde01, 0xe601, 0x0d02, 0x9901, 0xf504, 0xee01, 0xea01, 0xb601, 0x3b01, 0xbc03,
  0xe601, 0x3501, 0xbb01, 0x3c05, 0x9702, 0x3301, 0x4f01, 0x3802, 0xe801, 0xc201, 0x3a01, 0xec01, 0xf201, 0xed01, 0xb903,
  0x3701, 0xeb01, 0xef01, 0xf402, 0x9a01, 0x0c01, 0x9802, 0xb601, 0x3b03, 0xbc01, 0x3602, 0xe701, 0xbb03, 0xc401, 0x3c01,
  0xc302, 0x1501, 0x3801, 0xba02, 0xf202, 0xf301, 0xed01, 0xe901, 0x4e01, 0x3701, 0x2b02, 0xf303, 0xc603, 0x4b01, 0x0101,
  0x3606, 0xf203, 0xf501, 0x3505, 0xd801, 0x4401, 0xcc01, 0xc501, 0xee02, 0x4502, 0xf501, 0xf402, 0x0101, 0x3605, 0xef01,
  0xf204, 0x3505, 0xf303, 0x4401, 0x2c02, 0x1f01, 0x41fd, 0x2781, 0x9c02, 0x3802, 0x6803, 0x6704, 0x6604, 0x3afe, 0x1601,
  0x5b03, 0x5f01, 0x5e01, 0x2381, 0x3701, 0x1f81, 0x3801, 0x1702, 0x4302, 0xb103, 0x4101, 0xc504, 0x3f01, 0x370a, 0x7201,
  0x3811, 0x3d01, 0xab01, 0xc604, 0x4302, 0xaa01, 0x4001, 0xb204, 0xc801, 0x4201, 0x3f01, 0x2c02, 0x3701, 0x0d03, 0x9501,
  0x9901, 0x4502, 0x6e01, 0x4401, 0x1901, 0x3b01, 0x1b02, 0xc902, 0x0081, 0xbb02, 0x3c01, 0xdc03, 0x9701, 0x1f82, 0xfe01,
  0xd402, 0x1501, 0x4f02, 0x1181, 0xc303, 0x9801, 0xdb04, 0xc203, 0x1b01, 0x4502, 0xbc02, 0xc901, 0xff01, 0x0181, 0x4404,
  0x9a01, 0x3c02, 0x9601, 0x0c03, 0x4e01, 0x1a01, 0x5502, 0x5402, 0xc704, 0xc806, 0x9a06, 0x9802, 0xbc04, 0x0181, 0xbb05,
  0x9901, 0x3401, 0x9704, 0x5202, 0x1404, 0xd502, 0xd601, 0x6f01, 0x1504, 0x5302, 0x4f01, 0xba08, 0x8302, 0x0501, 0x8401,
  0xb90a, 0x3803, 0x0d01, 0xf401, 0x3b02, 0xd801, 0x7882, 0xde01, 0x3602, 0xca01, 0xdf03, 0xe002, 0x3501, 0x4982, 0xf305,
  0xbb01, 0xcf01, 0x3c03, 0xd001, 0x1201, 0x3702, 0x1301, 0xd001, 0xd701, 0x4401, 0xcf01, 0x0c01, 0x3b03, 0x0101, 0xbc01,
  0x3601, 0x3181, 0x3381, 0x3505, 0xd801, 0x6381, 0x6181, 0x3c02, 0xdf03, 0xe003, 0x4201, 0x3805, 0xf202, 0xf304, 0xca02,
  0x3702, 0x9801, 0xd301, 0x9701, 0x9801, 0xd301, 0x9701, 0x4f02, 0x3301, 0xba01, 0x1403, 0x7101, 0xc201, 0x6c02, 0xfd01,
  0x4b01, 0xb101, 0xfc05, 0x0901, 0x1505, 0xb905, 0x4e01, 0x0d01, 0xd004, 0x4504, 0xcf02, 0xf501, 0xc502, 0xf401, 0x4401,
  0x5402, 0x5f01, 0x9601, 0x3b07, 0xbc05, 0x0181, 0xbb03, 0x5501, 0x3c07, 0xc605, 0x4201, 0xcd01, 0x5301, 0xf201, 0x4602,
  0xf303, 0xba01, 0x7bfd, 0x78fe, 0x79fe, 0xcf02, 0xb901, 0xf301, 0xf401, 0xcc02, 0xe902, 0xe502, 0x0c01, 0xde03, 0xf504,
  0x0d01, 0xd001, 0xe802, 0x77fe, 0xcb02, 0x79fe, 0xd201, 0xba01, 0xd601, 0xf202, 0xe502, 0x62fe, 0x61fe, 0xcd01, 0xdd02,
  0x64fe, 0xb902, 0x9901, 0xcf01, 0xd207, 0x7481, 0xd101, 0xf201, 0x4881, 0x4681, 0x4781, 0x4581, 0x4c82, 0xfe01, 0x2d81,
  0x2e81, 0x2f82, 0x3081, 0x7481, 0x7581, 0x4881, 0xf301, 0xd10c, 0x9a01, 0xd201, 0xde01, 0xba03, 0x7a81, 0xd003, 0xce01,
  0x7882, 0xd501, 0x2f81, 0x3182, 0x7481, 0xcd01, 0xd101, 0x5d81, 0x4781, 0x4581, 0x9602, 0xd701, 0x7681, 0xb904, 0xba07,
  0xc602, 0x2f81, 0xd201, 0x3181, 0xca01, 0x5f81, 0x4981, 0x4401, 0x6181, 0xcd01, 0xb903, 0xcf01, 0x7b81, 0xce06, 0x7781,
  0x7881, 0x7981, 0xcf01, 0xd201, 0x7481, 0x7581, 0x7681, 0xcd01, 0xf401, 0xdf01, 0x6081, 0x4581, 0x3c01, 0x9701, 0xeb01,
  0xd001, 0xf501, 0xd101, 0x9801, 0x3b01, 0x2d81, 0x5e81, 0x5f81, 0x5d81, 0x6281, 0x6081, 0x6181, 0x3c01, 0x6481, 0xce01,
  0xe001, 0xcd04, 0x7b81, 0x7781, 0x7981, 0xed01, 0xd602, 0x77fc, 0xbc01, 0x75fc, 0xc701, 0x5efc, 0xc801, 0x62fc, 0xbb01,
  0x60fc, 0x7f01, 0xc201, 0x74fb, 0xd601, 0x76fb, 0x47fb, 0x45fb, 0x2df9, 0x2ff9, 0x5df9, 0xd601, 0x7b01, 0x3b01, 0x2efd,
  0x30fd, 0x32fd, 0x48fd, 0xf301, 0x46fd, 0x3c01, 0xd401, 0xd302, 0xd501, 0x7c01, 0xb601, 0xd003, 0x4504, 0x7481, 0x7681,
  0x4404, 0x4781, 0xfb01, 0xc301, 0xda01, 0x4503, 0x4403, 0xcf02, 0x0d01, 0x0601, 0xd003, 0xf001, 0xd103, 0xfc02, 0xcf01,
  0x1901, 0xdb03, 0x3b02, 0x3182, 0x3501, 0x0001, 0x6182, 0x3102, 0xbf01, 0xdc01, 0x5201, 0x2b02, 0x3802, 0xe801, 0x1701,
  0xec03, 0x8301, 0x4a01, 0xed01, 0x3801, 0x7a81, 0xd002, 0x3201, 0x4b01, 0x0101, 0x4981, 0x3601, 0x0f01, 0xc901, 0x8401,
  0xfd02, 0x7881, 0x4201, 0x3c03, 0x3901, 0xd205, 0xcf04, 0xdc01, 0x2c02, 0x1a01, 0xd001, 0xd903, 0x4582, 0x7482, 0x4783,
  0xed01, 0x2d82, 0xd901, 0x2f82, 0x3182, 0x5d82, 0xda01, 0xd501, 0xc606, 0x3506, 0xee01, 0xda01, 0xc503, 0x0c01, 0xc605,
  0x0d01, 0x9501, 0xce01, 0xd901, 0x3603, 0xd701, 0xc504, 0x1901, 0xc60c, 0xd102, 0x3510, 0xd802, 0xf308, 0xf103, 0xc502,
  0x9601, 0x0c01, 0xc602, 0x360d, 0xd201, 0xf002, 0xf206, 0xc505, 0x0684, 0x0782, 0x3504, 0x0881, 0x3603, 0x5602, 0x5701,
  0x3501, 0x3601, 0x5603, 0xe501, 0x3b01, 0x1702, 0x3603, 0xbe01, 0xca03, 0xd701, 0x3501, 0x6f01, 0x3804, 0x0601, 0xec01,
  0xbe01, 0xcc02, 0x6c03, 0x3503, 0xdb02, 0xcb02, 0xe401, 0x0c02, 0xdc01, 0x3707, 0xbb01, 0xbc01, 0xbb01, 0xbc01, 0xec01,
  0xbb01, 0x0e01, 0xbc01, 0xed01, 0x3801, 0xea01, 0xe501, 0xbb01, 0x3701, 0xeb01, 0xbc01, 0xdb01, 0xbb01, 0xe001, 0x4501,
  0x3601, 0x4a83, 0x4882, 0x4401, 0x4782, 0x4582, 0x4682, 0xdd01, 0xd303, 0xbb01, 0x1901, 0xbc01, 0x4502, 0x2f81, 0x3081,
  0x4881, 0x3282, 0x3501, 0x4401, 0xdf03, 0xd304, 0x2e81, 0x4c81, 0xe002, 0xa701, 0xd605, 0x5e81, 0x5f81, 0x5d81, 0x6281,
  0x6081, 0x6181, 0xdd03, 0x6481, 0x7781, 0xde04, 0x4501, 0x7a81, 0x7481, 0x7581, 0x7681, 0x4401, 0xd603, 0xdf03, 0x7981,
  0x3481, 0x5e81, 0xba01, 0xbc01, 0x3081, 0xca01, 0x3281, 0xf502, 0x4a81, 0x4b81, 0x4881, 0x4981, 0x4681, 0x4781, 0x6081,
  0xcd01, 0xb901, 0xbb01, 0x3481, 0x1f81, 0xba01, 0xce01, 0xbc01, 0x3081, 0x3181, 0x3281, 0xca01, 0x4a81, 0x4881, 0x7581,
  0xb902, 0xf501, 0xba08, 0xb501, 0xce01, 0x7781, 0xcb02, 0x7981, 0xbc02, 0xca01, 0x7581, 0x7681, 0x5f81, 0xd101, 0xf304,
  0x6281, 0xb903, 0xba05, 0x7a81, 0xd201, 0xf202, 0x5e81, 0x5f81, 0x0e81, 0xf401, 0xbb02, 0x6081, 0xcc02, 0xcd01, 0xb907,
  0x7b02, 0x2e82, 0x2f82, 0x3082, 0xc703, 0xd401, 0xd501, 0xd201, 0xff01, 0xc803, 0x4682, 0x4783, 0x4583, 0xd301, 0x7c01,
  0xd501, 0xd602, 0xc703, 0x5e82, 0x5f83, 0x5d82, 0xd401, 0x6082, 0x7782, 0x7e01, 0x7482, 0x7582, 0x7682, 0xc803, 0xd101,
  0xd602, 0xd502, 0x5201, 0xe801, 0xce01, 0x9901, 0xbe01, 0xef01, 0x7d01, 0xfc02, 0x3701, 0x9701, 0x3801, 0xfd04, 0x9801,
  0xd001, 0x0101, 0xcf01, 0xbe01, 0xd701, 0x8401, 0x9a01, 0xe902, 0xee01, 0x7e03, 0x5301, 0x2b01, 0x3305, 0xc203, 0x3b04,
  0x4301, 0xd901, 0xbd01, 0xb908, 0x3701, 0x3802, 0xba03, 0xe901, 0xc803, 0xd001, 0xc302, 0x8401, 0xda02, 0x9a01, 0x3c02,
  0x3404, 0x3701, 0xc201, 0xce04, 0xd106, 0xf201, 0x3501, 0xf301, 0xbb01, 0x3c02, 0x3b02, 0x3601, 0xbc01, 0xcd03, 0xf204,
  0xc301, 0x3c02, 0xd205, 0x3801, 0xc201, 0xd202, 0xed01, 0x3601, 0xca01, 0xf504, 0xcd04, 0xbb02, 0xc401, 0x7f01, 0x3c05,
  0xb901, 0xeb01, 0xba01, 0xce06, 0x3b06, 0x8001, 0xbc01, 0xd104, 0x3502, 0xf403, 0xec01, 0xc301, 0x1f82, 0x7b83, 0x6609,
  0x7a83, 0x3503, 0x6383, 0x5702, 0x6483, 0x8485, 0x8585, 0x3601, 0x3501, 0x6d85, 0x6e85, 0xb002, 0x8785, 0x0f81, 0x8685,
  0x3607, 0x6b0a, 0x7085, 0x3501, 0x6f85, 0x2284, 0xab03, 0xaa02, 0x0481, 0x6504, 0x1081, 0x3601, 0x6b02, 0x1e02, 0x7185,
  0x7285, 0x7385, 0xa301, 0xa401, 0x5d03, 0x5e05, 0x238b, 0x2283, 0x8b85, 0x8985, 0x8a85, 0x8885, 0x8702, 0x1f81, 0x7d85,
  0x660d, 0x7a85, 0x0e82, 0x6685, 0x3505, 0x248b, 0x6385, 0x0782, 0xe103, 0x7f85, 0x1c01, 0x7d85, 0x7c85, 0x670b, 0x6612,
  0x3602, 0x2583, 0x6685, 0x3501, 0x6885, 0x0881, 0x2483, 0x6585, 0xe102, 0x1f02, 0x7e86, 0x2683, 0x1282, 0x7f86, 0x6715,
  0x3602, 0x6786, 0x5a01, 0x3501, 0x6886, 0x3901, 0xe101, 0x2002, 0x7e85, 0x2782, 0x8185, 0x680d, 0x6705, 0x3603, 0x6785,
  0x5a02, 0x3505, 0x6a85, 0x8085, 0x2781, 0x8185, 0x6818, 0x2882, 0x3602, 0x6985, 0x3506, 0x6a85, 0x1482, 0x8084, 0x8c84,
  0x8284, 0x2201, 0x6803, 0x3604, 0x6984, 0x690c, 0x1281, 0x2989, 0x3502, 0x6b84, 0x6c84, 0x8384, 0x3801, 0x2301, 0x8286,
  0x2a81, 0x3604, 0x0c81, 0x6915, 0x2985, 0x350a, 0x0d81, 0x6b86, 0x1581, 0x6c86, 0xab02, 0x1883, 0x1783, 0x9e01, 0x8386,
  0x3801, 0x1781, 0xab01, 0x8285, 0x2b8a, 0x8585, 0x6e01, 0x6d01, 0x2981, 0x3501, 0x6b85, 0x6a0f, 0x6e85, 0x1884, 0x6903,
  0xb601, 0x1f83, 0x0d01, 0x0283, 0x7b81, 0x6601, 0x7a81, 0x6c01, 0xd702, 0xf501, 0x1184, 0x2481, 0x4404, 0x6381, 0x0781,
  0xc502, 0xdd09, 0x6481, 0x6002, 0xde02, 0xe101, 0x3701, 0x3801, 0x2282, 0xb601, 0x8785, 0x8685, 0x6b09, 0x7085, 0x4401,
  0x6f85, 0xaa01, 0x2181, 0x3702, 0x3801, 0x8b85, 0x8985, 0x8a85, 0x8885, 0x6b07, 0xab01, 0x7185, 0x7285, 0x7385, 0x0582,
  0x3702, 0x6201, 0x7f84, 0x7d84, 0x6701, 0x6614, 0x1a81, 0x6384, 0x0681, 0x6584, 0x3801, 0x2583, 0x2489, 0x7c84, 0xc201,
  0x7a84, 0x1281, 0x6684, 0x0e84, 0x6884, 0x1682, 0x1481, 0x3703, 0x3801, 0x7e86, 0x7f86, 0x6725, 0x8186, 0x6786, 0x1181,
  0x6886, 0x6a86, 0x5f01, 0x0b81, 0x0881, 0x3801, 0x7e85, 0x8185, 0x6810, 0x6706, 0x6785, 0x6a85, 0x1481, 0x5f01, 0x6201,
  0x3701, 0x8086, 0x8186, 0x681b, 0x1181, 0x6986, 0x1588, 0x6a86, 0x1381, 0x3701, 0x3802, 0x8c86, 0x8386, 0x8286, 0x2881,
  0x691e, 0x5901, 0x2988, 0x0c81, 0x6b86, 0x1481, 0x6c86, 0x1681, 0x3701, 0x3802, 0x8285, 0x6902, 0x1182, 0x6b85, 0x3702,
  0x3803, 0x8485, 0x8785, 0x2b8a, 0x8585, 0x6a1e, 0x0c83, 0x0d81, 0x6d85, 0x7085, 0x6e85, 0x1886, 0x2081, 0x3707, 0x3806,
  0x1783, 0x8486, 0x8786, 0x3604, 0x0c83, 0x6a08, 0x6d01, 0x3505, 0x0d84, 0x6d86, 0x7086, 0xaa02, 0x2085, 0x3701, 0x1f85,
  0x2581, 0x8c86, 0x2785, 0xb602, 0x2881, 0x6806, 0x6706, 0x6607, 0x6e01, 0x4602, 0x1a89, 0x1b83, 0x2481, 0x2682, 0x1f82,
  0x8c8a, 0x2785, 0xab01, 0x2881, 0x6809, 0x6704, 0x6606, 0x6e01, 0x198a, 0x1b86, 0xb602, 0x2685, 0x2482, 0x6201, 0xb401,
  0x6e01, 0x7201, 0x6805, 0x6704, 0x6602, 0x690d, 0xb201, 0x1984, 0x1a83, 0x1c81, 0x1d82, 0x2581, 0x2481, 0x1f81, 0xb001,
  0x8c8b, 0x2784, 0x2682, 0x2985, 0x2884, 0x4601, 0xb404, 0x8c87, 0x4101, 0x1981, 0x1b85, 0x1d82, 0x3e01, 0x3701, 0x8c82,
  0x7101, 0xfa03, 0x6e02, 0x1b83, 0x3d01, 0x1c82, 0x9601, 0x3e01, 0xb601, 0x8c81, 0x2487, 0x7bfb, 0xc202, 0x78fb, 0x6605,
  0x6e05, 0x1981, 0xf301, 0x61fb, 0x0681, 0xc001, 0x64fb, 0x0282, 0x3801, 0x84f7, 0x87f7, 0x2b85, 0x6e04, 0x6a02, 0x6df7,
  0x70f7, 0x1882, 0x87f7, 0x0f81, 0x86f7, 0x3602, 0x70f7, 0x6ff7, 0x0481, 0x89fb, 0x1081, 0x0f81, 0x86fb, 0x6ffb, 0x71fb,
  0x0483, 0x0582, 0x8bf7, 0xaa01, 0x89f7, 0x8af7, 0x88f7, 0x3602, 0xb102, 0x71f7, 0x72f7, 0x73f7, 0x0581, 0xab01, 0x1f87,
  0x1c01, 0x7df9, 0x1183, 0x6603, 0x7af9, 0x2582, 0x66f9, 0x63f9, 0x0689, 0x7ff9, 0x2482, 0x7cf9, 0xb101, 0x1182, 0x68f7,
  0x0782, 0x65f9, 0x2681, 0xb201, 0x7ef7, 0x2781, 0x7ff7, 0x6702, 0x2581, 0x67f7, 0x68f7, 0x2681, 0x81ef, 0x2882, 0x3601,
  0xb201, 0x6aef, 0x1382, 0x0982, 0x80f9, 0x2782, 0x83f9, 0x2981, 0x81f9, 0x69f9, 0x3501, 0x6af9, 0x1482, 0x6cf9, 0x0a81,
  0x8cf1, 0x83f9, 0x82f9, 0x2881, 0x0d81, 0x6bf9, 0x6cf9, 0x0c85, 0x6bee, 0x82ee, 0x6902, 0x84f6, 0x85f6, 0x2085, 0x6df6,
  0x6ef6, 0x9e01
};
