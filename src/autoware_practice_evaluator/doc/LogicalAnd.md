# LogicalAnd

## 処理内容

以下の定義にしたがって入力ノードの状態を統合します。入力ノードが空の場合は `success` を返します。

|           | `failure` | `judging` | `success` |
| --------- | --------- | --------- | --------- |
| `failure` | `failure` | `failure` | `failure` |
| `judging` | `failure` | `judging` | `judging` |
| `success` | `failure` | `judging` | `success` |

## 書式

| Field | Description                |
| ----- | -------------------------- |
| type  | LogicalAnd                 |
| list  | 入力となるノードのリスト。 |

## 記載例

作成中
