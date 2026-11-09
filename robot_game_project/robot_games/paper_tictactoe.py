"""Robot-first X / human O, camera-observed physical 3x3 paper board.

Two sources:
  --source demo: virtual paper with human input; exercises full turn state machine.
  --source camera: OpenCV webcam, operator-assisted captures, four-corner perspective correction.

In camera mock, an operator must draw the robot's X on paper because MockRobot
never actuates a pen. Never proceed on ambiguous observations.
"""
from __future__ import annotations
import argparse
import time
from pathlib import Path
import cv2
import numpy as np
from .tictactoe import best_move, outcome, render_board
from .motion import MockRobot, GuardedDoosan, image_path_to_base

SIDE = 600
ROOT = Path(__file__).resolve().parents[1]


def x_strokes(row, col, side=SIDE, arm=50):
    x, y = (2*col+1)*side/6, (2*row+1)*side/6
    return [[(round(x-arm), round(y-arm)), (round(x+arm), round(y+arm))],
            [(round(x+arm), round(y-arm)), (round(x-arm), round(y+arm))]]


def get_frame(cap):
    for _ in range(4):
        ok, frame=cap.read()
    if not ok: raise RuntimeError('Camera read failed')
    return frame


def camera_corners(frame):
    """Select the *outer four corners* of the 3x3 board in TL,TR,BR,BL order."""
    corners=[]
    screen='Select BOARD corners: TL -> TR -> BR -> BL; ESC cancels'
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and len(corners)<4:
            corners.append((float(x),float(y)))
    cv2.namedWindow(screen,cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(screen,click)
    try:
        while len(corners)<4:
            preview=frame.copy()
            for i,p in enumerate(corners):
                cv2.circle(preview,tuple(map(int,p)),6,(0,0,255),-1)
                cv2.putText(preview,str(i+1),tuple(map(int,p)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,255),2)
            cv2.imshow(screen,preview)
            if cv2.waitKey(30)&0xff==27:raise KeyboardInterrupt('Calibration canceled')
    finally:cv2.destroyWindow(screen)
    src=np.asarray(corners,np.float32)
    dst=np.asarray([(0,0),(SIDE-1,0),(SIDE-1,SIDE-1),(0,SIDE-1)],np.float32)
    H=cv2.getPerspectiveTransform(src,dst)
    return H


def warp(frame,H):
    return cv2.warpPerspective(frame,H,(SIDE,SIDE))


def capture(cap,H,label,output_dir):
    """Press SPACE to capture an image after the pen is out of the camera view."""
    title='Capture (SPACE) / quit (Q)'
    while True:
        frame=get_frame(cap)
        img=warp(frame,H)
        preview=img.copy()
        cv2.putText(preview,label[:32],(12,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,90,255),2)
        cv2.imshow(title,preview)
        key=cv2.waitKey(30)&0xff
        if key in (ord('q'),27): raise KeyboardInterrupt('Operator quit')
        if key==32:
            cv2.imwrite(str(output_dir/f'capture_{int(time.time()*1000)}.png'),img)
            return img


def cell_ink(img,row,col,threshold=135):
    """Crop *inside* cell so grid boundaries never count as moves."""
    g=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) if img.ndim==3 else img
    g=cv2.GaussianBlur(g,(3,3),0)
    x0=col*200+35;x1=(col+1)*200-35
    y0=row*200+35;y1=(row+1)*200-35
    return cv2.threshold(g[y0:y1,x0:x1],threshold,255,cv2.THRESH_BINARY_INV)[1]


def classify_o(roi):
    """Distinguish a ring from X by connected ink around an empty center.
    Accept only clean printed/pen circle; no general-purpose handwriting classifier.
    """
    h,w=roi.shape
    density=cv2.countNonZero(roi)/roi.size
    if not .025<density<.48:return False
    center=roi[h//2-12:h//2+12,w//2-12:w//2+12]
    if cv2.countNonZero(center)/center.size>.12:return False
    # Ink on four sides of ring and not in middle; tolerate hand-drawn circles.
    pts=[roi[8:38,w//2-15:w//2+15],roi[-38:-8,w//2-15:w//2+15],
         roi[h//2-15:h//2+15,8:38],roi[h//2-15:h//2+15,-38:-8]]
    return sum(cv2.countNonZero(p)>8 for p in pts)>=3


def detect_human_o(before,after,board,min_added=160):
    """Compare same top-down board image; exactly one EMPTY cell may gain O ink.
    Returns (row,col), or raises ValueError without changing board state.
    """
    changes=[]
    for r in range(3):
        for c in range(3):
            if board[r][c]!=' ': continue
            a=cell_ink(before,r,c);b=cell_ink(after,r,c)
            added=cv2.bitwise_and(b,cv2.bitwise_not(a))
            # Suppress small lighting noise
            added=cv2.morphologyEx(added,cv2.MORPH_OPEN,np.ones((2,2),np.uint8))
            count=cv2.countNonZero(added)
            if count>min_added:
                if not classify_o(b):
                    raise ValueError(f'New marks in ({r+1},{c+1}) are not confidently O; retry')
                changes.append((r,c))
    if len(changes)!=1:
        raise ValueError(f'Expected exactly one new O on an empty cell; detected {changes}')
    return changes[0]


def print_board(board):
    print('\n'+'\n---------\n'.join(' | '.join(board[r][c] if board[r][c]!=' ' else str(r*3+c+1)
                                       for c in range(3)) for r in range(3))+'\n')


def choose_robot_x(board):
    return best_move(board,robot='X',human='O')


def save(board,outdir,turn):
    cv2.imwrite(str(outdir/f'paper_tic_turn_{turn:02d}.png'),render_board(board))
    cv2.imwrite(str(outdir/'paper_tic_final.png'),render_board(board))


def robot_draw_x(robot,config,move,physical=False):
    for index,stroke in enumerate(x_strokes(*move)):
        if physical:
            trajectory=image_path_to_base(stroke,(SIDE,SIDE),config['robot'])
        else:trajectory=stroke
        robot.draw(trajectory,f'X stroke {index+1} cell {move[0]+1},{move[1]+1}')


def play(source='demo',camera_index=0,robot_mode='mock',config_path=None,
         execute_motion=False,output_dir=None,moves=None):
    if robot_mode=='real' and source!='camera':
        raise ValueError('Physical robot requires actual camera observations; demo mode prohibited')
    if robot_mode=='real' and not execute_motion:
        raise PermissionError('Real arm requires --execute-motion and approved configuration')
    cfg=None
    if config_path:
        import yaml
        cfg=yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
    if robot_mode=='real' and (not cfg or not cfg['robot'].get('armed') or not cfg['robot'].get('calibration_confirmed')):
        raise PermissionError('Real arm requires manually calibrated, armed robot config')
    if robot_mode=='real':
        raise NotImplementedError('Real paper game blocked: physical pen-up Z/contact, H-to-base registration and motion feasibility have not been validated. Use mock for camera trials.')
    robot=MockRobot()
    outdir=Path(output_dir or ROOT/'out');outdir.mkdir(parents=True,exist_ok=True)
    board=[[' ']*3 for _ in range(3)]
    history=[];cap=None;H=None;before=None;turn=0;result='QUIT'
    user_moves=iter(moves) if moves is not None else None
    try:
        if source=='camera':
            cap=cv2.VideoCapture(camera_index)
            if not cap.isOpened(): raise RuntimeError(f'Cannot open webcam index {camera_index}')
            print('[CAMERA] Show a clean empty paper tic-tac-toe grid.')
            H=camera_corners(get_frame(cap))
            before=capture(cap,H,'EMPTY BOARD - press SPACE',outdir)
        save(board,outdir,turn)
        while outcome(board) is None:
            print_board(board)
            # ROBOT X turn: after draw, always obtain verified image baseline.
            move=choose_robot_x(board)
            if move is None:break
            r,c=move
            robot_draw_x(robot,cfg,move)
            if source=='camera':
                print(f'[MOCK] Robot X planned at row={r+1}, col={c+1}.')
                print('[OPERATOR] Since the mock does not physically draw, manually draw X at that location, then SPACE to capture.')
                observed=capture(cap,H,'DRAW ROBOT X MANUALLY; SPACE',outdir)
                # Prevent silently trusting a fictitious robot move.
                added=cv2.countNonZero(cv2.bitwise_and(cell_ink(observed,r,c),
                                  cv2.bitwise_not(cell_ink(before,r,c))))
                if added<160: raise RuntimeError('Expected robot X was not visible; stopping game')
                before=observed
            board[r][c]='X';history.append(('X',r,c));turn+=1;save(board,outdir,turn)
            if outcome(board) is not None:break
            if source=='demo':
                while True:
                    try:raw=str(next(user_moves)) if user_moves is not None else input('Human O position (1..9 or q): ')
                    except (EOFError,StopIteration,KeyboardInterrupt):return 'QUIT',history
                    if raw.strip().lower() in ('q','quit','exit'):return 'QUIT',history
                    if raw.strip().isdigit() and 1<=int(raw)<=9:
                        rr,cc=divmod(int(raw)-1,3)
                        if board[rr][cc]==' ':break
                    print('[INVALID] Enter an empty cell 1..9')
            else:
                print('[HUMAN] Draw ONE circle O in an empty cell. Do not move the paper/camera. SPACE to capture.')
                rr=cc=None
                while True:
                    after=capture(cap,H,'HUMAN O; SPACE to capture',outdir)
                    try:
                        rr,cc=detect_human_o(before,after,board)
                        print(f'[VISION] O detected at row={rr+1}, col={cc+1}')
                        # User must verify recognition before accepting a move.
                        confirmation=input('Accept recognized O? [y/N/q] ').strip().lower()
                        if confirmation=='q':return 'QUIT',history
                        if confirmation!='y':continue
                        before=after;break
                    except ValueError as e:print('[VISION RETRY]',e)
            board[rr][cc]='O';history.append(('O',rr,cc));turn+=1;save(board,outdir,turn)
        result=outcome(board) or 'QUIT'
        print_board(board)
        print('[RESULT]',{'X':'ROBOT X WINS','O':'HUMAN O WINS','DRAW':'DRAW','QUIT':'QUIT'}[result])
        if result=='X':robot.celebrate()
        return result,history
    except KeyboardInterrupt:
        print('[STOP] Operator requested stop. No further move.');return 'QUIT',history
    finally:
        if cap is not None:cap.release();cv2.destroyAllWindows()
        robot.home()


def main():
    p=argparse.ArgumentParser(description='Paper tic-tac-toe: robot X first, human O, optional camera')
    p.add_argument('--source',choices=['demo','camera'],default='demo')
    p.add_argument('--camera-index',type=int,default=0)
    p.add_argument('--robot-mode',choices=['mock','real'],default='mock')
    p.add_argument('--config',default=None)
    p.add_argument('--execute-motion',action='store_true')
    p.add_argument('--output-dir',type=Path,default=None)
    p.add_argument('--moves',nargs='*',type=int,help='Scripted human O cells for demo')
    args=p.parse_args()
    play(args.source,args.camera_index,args.robot_mode,args.config,args.execute_motion,args.output_dir,args.moves)

if __name__=='__main__': main()
