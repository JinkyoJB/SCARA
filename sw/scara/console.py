"""콘솔 출력 보조.

윈도우 콘솔의 기본 코드페이지가 cp949 라서, 한글은 나오지만
`—`(em dash), `★`, `→` 같은 문자에서 `UnicodeEncodeError` 로 죽는다.

인코딩 자체를 UTF-8로 바꾸면 cp949 콘솔에서는 오히려 한글이 깨지므로,
**인코딩은 그대로 두고 오류 처리만 `replace`** 로 바꾼다.
표현 못 하는 문자는 `?` 로 나오되 프로그램은 죽지 않는다.
"""

import sys


def setup() -> None:
    """스크립트 맨 앞에서 한 번 호출한다."""
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if callable(reconfigure):
            try:
                reconfigure(errors="replace")
            except (ValueError, OSError):
                pass  # 리다이렉트된 스트림 등
