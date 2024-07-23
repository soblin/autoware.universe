# Autoware Universe Code Reading

Autoware Universeのコードを解説するドキュメントサイト

## version

`a156e0da74c78f8d48be59b6210446ef745614a0`

## nomenclature

| 表記              | タイプ                   | 例                                         |
| ----------------- | ------------------------ | ------------------------------------------ |
| **_太斜字_**      | **_機能に特有の用語_**   | **_ShiftPullOver_**                        |
| **太字**          | **関数名**               | **Node::spin_some()**                      |
| _斜体_            | _パッケージ名_           | _autoware_launch_                          |
| `コードブロック`  | `ソースコード`，`変数名` | `data = msg->position`, `is_in_transition` |
| `$コードブロック` | `機能のパラメーター`     | `$ekf.weight`                              |

## adomonitions

/// tip | 注意点
コードを理解する上で注意したほうが良い点
///

/// info | 関連情報
実装の背景にある関連情報
///

/// question | 疑問点
読んでいて疑問に感じた点
///

/// bug | バグ
意図しないバグを含んでいる部分の解説
///
