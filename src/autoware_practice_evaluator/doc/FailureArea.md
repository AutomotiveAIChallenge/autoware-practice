# FailureArea

## 処理内容

車両がエリアの中に入っていれば `failure` を返し、入っていなければ `judging` を返します。

## 書式

| Field | Description                          |
| ----- | ------------------------------------ |
| type  | FailureArea                          |
| area  | エリアの頂点座標の配列。半時計回り。 |

## 記載例

```yaml
type: FailureArea
area:
  - [16.0, +5.0]
  - [14.0, +5.0]
  - [14.0, -5.0]
  - [16.0, -5.0]
```
