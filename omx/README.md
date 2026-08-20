# OMX 로봇팔

로봇팔·카메라가 연결된 **별도 PC**에서 OMX 픽업 서버를 띄우고,
관제 PC(`server/`)는 `OMX_URL`로 HTTP 픽업만 요청한다.

## 배치

```
관제 PC (controller :4100)  ──LAN HTTP──>  OMX PC (omx server :8080)  ──>  로봇팔
주행로봇 PC (pinky :4200)   ──LAN HTTP──>  관제 PC
```

## 빠른 시작 (OMX PC)

```bash
cd omx/OMX_service_TS_Project
cp .env.example .env
POLICY=/path/to/checkpoint ./scripts/start_server.sh
```

기동 로그에 `OMX_URL=http://<LAN IP>:8080` 예시가 출력된다.  
그 URL을 **관제 PC** `server/.env`의 `OMX_URL`에 넣고 `ADAPTER_MODE=mock`을 끈다.

## 문서

- [`OMX_service_TS_Project/README.md`](OMX_service_TS_Project/README.md) — 설치·롤아웃
- [`OMX_service_TS_Project/API.md`](OMX_service_TS_Project/API.md) — HTTP API 규격
