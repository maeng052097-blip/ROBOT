"""웨이퍼 이송셀 CELL_V9 공정 모니터 패키지.

재활용 로봇(common/ config.py)과는 별개 기계이므로 자체 설정(cell_config.py)을 쓴다.
순수 로직(sequence / geometry / tof / sim)은 cv2·numpy 없이 동작한다(단위테스트 대상).
"""
