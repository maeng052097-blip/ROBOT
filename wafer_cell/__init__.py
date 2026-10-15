"""웨이퍼 검사셀 R9 공정 모니터 패키지 (이전 CELL_V9 는 git 이력에만 남음).

재활용 로봇(common/ config.py)과는 별개 기계이므로 자체 설정(cell_config.py)을 쓴다.
순수 로직(sequence / geometry / sim)은 cv2·numpy 없이 동작한다(단위테스트 대상).
"""
