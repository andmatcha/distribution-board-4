    // CAN動作確認用: 各モーターへ順番に信号を送信
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    // 1. サーボモーター: 90度
    TxHeader.StdId = 0x100;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 2;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxData[0] = 0x03; // 900 (90.0度) の上位バイト
    TxData[1] = 0x84; // 900 の下位バイト
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(500);

    // 2. DCモーター1: 正転50%
    TxHeader.StdId = 0x101;
    TxData[0] = 1;  // direction: 正転
    TxData[1] = 50; // duty: 50%
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(500);

    // 3. DCモーター2: 正転30%
    TxHeader.StdId = 0x102;
    TxData[0] = 1;  // direction: 正転
    TxData[1] = 30; // duty: 30%
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(500);

    // 4. 全モーター停止
    TxHeader.StdId = 0x101;
    TxData[0] = 0;  // direction: 停止
    TxData[1] = 0;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(100);

    TxHeader.StdId = 0x102;
    TxData[0] = 0;  // direction: 停止
    TxData[1] = 0;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(1000);
