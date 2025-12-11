# distribution-board-2

## コマンド

- アップロード
```
pio run -t upload
```

- アップロード(ポート指定)
```
pio run -t upload --upload-port ポート名
```

- printfの出力を表示
```
./stlink_monitor.zsh
```

- 接続を確認
```
st-info --probe
```

## 使用パーツ

- STM32F103TBU6 (マイコン)
- SN65HVD75DGKR (半二重 RS-485 トランシーバ)
- AMT21 (エンコーダー)
- MCP2562FD-E/P (CANトランシーバ)

