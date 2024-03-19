# autoware_practice_evaluator

Autoware 入門講座で使用する汎用判定機です。

## 判定の仕組み

判定条件を設定ファイルに木構造で記載していきます。木構造の各ノードは以下の判定結果を持っており、タイマーで一定時間ごとに状態を更新しています。

- `Judging`
- `Success`
- `Failure`

## 判定ノードの処理

- [`LatchResult`](./doc/LatchResult.md)
- [`JudgeResult`](./doc/JudgeResult.md)
- [`SuccessArea`](./doc/SuccessArea.md)
- [`FailureArea`](./doc/FailureArea.md)
- [`LogicalAnd`](./doc/LogicalAnd.md)
