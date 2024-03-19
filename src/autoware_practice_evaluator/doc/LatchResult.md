# LatchResult

## 処理内容

入力ノードの状態が `judging` 以外であれば判定結果を固定します。基本的にこれを最上位に配置して最初に成立した判定で結果を確定させます。

## 書式

| Field | Description        |
| ----- | ------------------ |
| type  | LatchResult        |
| node  | 入力となるノード。 |

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
