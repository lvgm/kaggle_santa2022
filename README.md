# Kaggle Santa2022

#### Competition
https://www.kaggle.com/competitions/santa-2022

#### Approach:
- use LKH to solve each quadrant
- use fixed configuration for each point, so the link can be moved from (i,j) to (i+1,j),(i-1,j),(i,j+1),(i,j-1),(i+1,j+1),(i+1,j-1),(i-1,j+1),(i-1,j-1),(i,j+2),(i+1,j+2),(i,j-2),(i-1,j-2) in the upper right quadrant - and other 3 quardants respectively - with minimal reconfiguration cost
- staring from (0,0) -> (1,0) -> (1,64) -> (0,65), ending from (0,64) -> (0,0)

This ranked 36/875

The arrow png is generated following `pixel travel map`

![arrow plot](plot.png)

#### Reference
- [geting start](https://www.kaggle.com/code/ryanholbrook/getting-started-with-santa-2022)
- [A recipe for getting below 76000](https://www.kaggle.com/competitions/santa-2022/discussion/376306)
- [pixel travel map](https://www.kaggle.com/code/oxzplvifi/pixel-travel-map)
- [LKH](http://webhotel4.ruc.dk/~keld/research/LKH/)
