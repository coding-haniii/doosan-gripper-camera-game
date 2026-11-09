import argparse
from pathlib import Path
import cv2
import yaml
from . import vision as vision_mod
from .vision import recognize_card, tic_board_from_image
from .tictactoe import best_move, outcome, render_board
from .maze import generate_maze, solve_maze, overlay
from .motion import MockRobot, GuardedDoosan, image_path_to_base

ROOT=Path(__file__).resolve().parents[1]
def cfg_load(path):
    with open(path,encoding='utf8') as f: return yaml.safe_load(f)

def tictactoe(args,robot,cfg):
    board=[[' ',' ',' '],[' ','X',' '],[' ',' ',' ']]
    if args.image:
        img=cv2.imread(args.image)
        if img is None: raise ValueError(f'Could not load {args.image}')
        board=tic_board_from_image(img)
    winner=outcome(board)
    move=best_move(board) if winner is None else None
    if move:
        r,c=move; board[r][c]='O'
        # Pixel path for O stroke, preserving pen-up between separate actions.
        import numpy as np
        center=((2*c+1)*100,(2*r+1)*100)
        path=[(int(center[0]+65*np.cos(t)),int(center[1]+65*np.sin(t))) for t in np.linspace(0,2*np.pi,64)]
        if isinstance(robot,MockRobot): robot.draw(path,'tic-tac-toe O')
        else: robot.draw(image_path_to_base(path,(900,900),cfg['robot']),'tic-tac-toe O')
    final=outcome(board)
    out=ROOT/'out';out.mkdir(exist_ok=True)
    cv2.imwrite(str(out/'tictactoe_result.png'),render_board(board))
    print('Board:');print('\n'.join(' | '.join(row) for row in board));print('Winner:',final,'Move:',move)
    if final=='O': robot.celebrate()
    robot.home()

def maze(args,robot,cfg):
    if args.image:
        img=cv2.imread(args.image,cv2.IMREAD_GRAYSCALE)
        if img is None:raise ValueError(f'Could not load {args.image}')
        if args.start is None or args.goal is None:
            raise ValueError('--start X Y and --goal X Y required for arbitrary maze')
        start,goal=tuple(args.start),tuple(args.goal)
    else:
        img,start,goal=generate_maze(args.seed,args.cells)
    path=solve_maze(img,start,goal,clearance=cfg['vision']['maze_clearance_px'],
                    threshold=cfg['vision']['maze_threshold'],stride=cfg['vision']['maze_step_px'])
    out=ROOT/'out';out.mkdir(exist_ok=True)
    cv2.imwrite(str(out/'maze_input.png'),img)
    cv2.imwrite(str(out/'maze_solved.png'),overlay(img,path))
    print(f'Maze solved: {len(path)} waypoints; start={start}; goal={goal}')
    if isinstance(robot,MockRobot):robot.draw(path,'maze')
    else:robot.draw(image_path_to_base(path,img.shape,cfg['robot']),'maze')
    robot.home()

def main():
    parser=argparse.ArgumentParser(description='Doosan E0509 camera games (mock-first)')
    parser.add_argument('--config',default=str(ROOT/'configs/default.yaml'))
    parser.add_argument('--mode',choices=['mock','real'],default='mock')
    parser.add_argument('--game',choices=['1','2','3','tictactoe','maze','exit','camera'],default='1')
    parser.add_argument('--image',help='Photo of game board or maze')
    parser.add_argument('--card-image',help='Photo of action card for recognition')
    parser.add_argument('--truth',choices=['tictactoe','maze','exit'],help='지금 보여주는 카드의 정답(학습용). out/ocr_log.jsonl 에 함께 기록됨')
    parser.add_argument('--start',type=int,nargs=2,metavar=('X','Y'))
    parser.add_argument('--goal',type=int,nargs=2,metavar=('X','Y'))
    parser.add_argument('--seed',type=int,default=7)
    parser.add_argument('--cells',type=int,default=8)
    parser.add_argument('--execute-motion',action='store_true',help='Additional physical motion gate')
    parser.add_argument('--paper-maze',action='store_true',help='maze/card 2: run chained camera->JSON->CSV->robot pipeline (robot_games.maze_game)')
    parser.add_argument('--quiet-ocr',action='store_true',help='OCR 디버그 로그/out/debug 이미지 저장 끄기')
    parser.add_argument('--paper-maze-source',choices=['camera','image','demo'],default='camera')
    args=parser.parse_args();cfg=cfg_load(args.config)
    if args.truth: vision_mod.TRUTH=args.truth
    if args.quiet_ocr: vision_mod.DEBUG=False
    game={'1':'tictactoe','2':'maze','3':'exit'}.get(args.game,args.game)
    if args.card_image:
        img=cv2.imread(args.card_image)
        if img is None:raise ValueError('Card image unreadable')
        game=recognize_card(img)
        if not game:raise ValueError('Card unrecognized: no action performed (out/debug/ 의 이미지와 [card] 로그를 확인하세요)')
        print('Card detected:',game)
    if game=='camera':
        cap=cv2.VideoCapture(cfg['camera']['index'])
        if not cap.isOpened():raise RuntimeError('Cannot open camera')
        cam=cfg.get('camera',{})
        if cam.get('width'):cap.set(cv2.CAP_PROP_FRAME_WIDTH,cam['width'])
        if cam.get('height'):cap.set(cv2.CAP_PROP_FRAME_HEIGHT,cam['height'])
        print('Show action card inside the green guide; press SPACE to capture, Q to quit')
        try:
            for _ in range(15):cap.read()     # 자동노출/초점 안정화를 위해 초기 프레임 버림
            while True:
                ok,frame=cap.read()
                if not ok:raise RuntimeError('Camera frame failed')
                # 가이드 박스: 화면 중앙 70% (카드를 화면의 30~60% 이상 채우고 평평하게 두기)
                view=frame.copy();fh,fw=view.shape[:2]
                cv2.rectangle(view,(int(fw*.15),int(fh*.15)),(int(fw*.85),int(fh*.85)),(0,200,0),2)
                cv2.imshow('Action card (SPACE capture/Q exit)',view)
                key=cv2.waitKey(20)&0xff
                if key==ord('q'):game='exit';break
                if key==32:
                    out=ROOT/'out';out.mkdir(exist_ok=True)
                    cv2.imwrite(str(out/'capture.png'),frame)   # 캡처 원본 저장 (재현/디버깅용)
                    game=recognize_card(frame)                  # 전체 프레임 전달: vision에서 카드 검출/크롭
                    if game:
                        print('Detected:',game)
                        # 카메라 모드에서 maze 카드가 인식되면 paper_maze 파이프라인 자동 활성화
                        if game == 'maze':
                            args.paper_maze = True
                        break
                    print('Card not recognized. Try again. (out/capture.png, out/debug/ 확인)')
        finally:
            cap.release();cv2.destroyAllWindows()
    if game=='maze' and args.paper_maze:      # 카메라 종이 미로: 비전→JSON→CSV→로봇 연속 실행
        from .maze_game import run as run_paper_maze
        argv=['--mode',args.mode,'--source',args.paper_maze_source,'--seed',str(args.seed)]
        if args.execute_motion: argv.append('--execute-motion')
        if args.image: argv+=['--image',args.image]
        raise SystemExit(run_paper_maze(argv))
    if args.mode=='real' and not args.execute_motion:
        raise PermissionError('Physical mode requires --execute-motion and separate calibrated+armed YAML')
    robot=MockRobot() if args.mode=='mock' else GuardedDoosan(cfg['robot'])
    try:
        if game=='tictactoe':tictactoe(args,robot,cfg)
        elif game=='maze':maze(args,robot,cfg)
        elif game=='exit':print('Exit card: safe exit');robot.home()
        else:raise ValueError('Unknown game, no motion performed')
    finally:
        if hasattr(robot,'close'):robot.close()

if __name__=='__main__':main()
