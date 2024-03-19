# JudgeResult

## 処理内容

入力ノードの中に `judging` 以外の状態があればそれを返します。もし `success` と `failure` が混ざっていれば `failure` を返します。

## 書式

| Field | Description                |
| ----- | -------------------------- |
| type  | JudgeResult                |
| list  | 入力となるノードのリスト。 |

## 記載例

```yaml
type: LatchResult
node:
  type: JudgeResult
  list:
    - { type: SuccessArea, area: ... }
    - { type: FailureArea, area: ... }
    - { type: FailureArea, area: ... }
    - { type: FailureArea, area: ... }
```
