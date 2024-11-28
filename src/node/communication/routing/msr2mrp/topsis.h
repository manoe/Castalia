//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2024                                        *
//*  Developed at Pazmany Peter Catholic University,                            *
//*               Faculty of Information Technology and Bionics                 *
//*  Author(s): Balint Aron Uveges                                              *
//*  This file is distributed under the terms in the attached LICENSE file.     *
//*                                                                             *
//*******************************************************************************


#ifndef _TOPSIS_H_
#define _TOPSIS_H_

#include <cmath>
#include <armadillo>

using namespace arma;

struct ps_alt {
    std::string sink;
    int pathid;
    double rank;
};

class TopsisEngine {
    private:
        mat table;
        int alt_num;
        int alt_added;
        std::vector<ps_alt> alts;
        rowvec weights;
        std::vector<bool> attrs;
    protected:
        mat normalizeMatrix() {
            mat pow_table = square(table);
            mat norm_table = table;
            rowvec norm(table.n_cols,fill::zeros);
            for(int i=0; i < table.n_cols ; ++i) {
                if(sum(pow_table.col(i)) == 0.0) {
                    norm(i)=1;
                } else {
                    norm(i)=sum(pow_table.col(i));
                }
            }
            norm=sqrt(norm);
            for(int i=0; i < table.n_rows ; ++i) {
                norm_table.row(i)=table.row(i)/norm;
            }
            return norm_table;
        };
        mat applyWeights(mat table) {
            mat norm_w_table = table;
            for(int i=0 ; i < table.n_rows ; ++i) {
                norm_w_table.row(i) = norm_w_table.row(i) % weights;
            }
            return norm_w_table;
        };
        rowvec calculateIdealBest(mat table) {
            rowvec v_best(table.n_cols);
            for(int i=0; i<table.n_cols; ++i) {
                if(attrs[i]) {
                    v_best[i]=max(table.col(i));
                } else {
                    v_best[i]=min(table.col(i));
                }
            }
            return v_best;
        };
        rowvec calculateIdealWorst(mat table) {
            rowvec v_worst(table.n_cols);
            for(int i=0; i<table.n_cols; ++i) {
                if(attrs[i]) {
                    v_worst[i]=min(table.col(i));
                } else {
                    v_worst[i]=max(table.col(i));
                }
            }
            return v_worst;
        };
        colvec calculateSeparation(mat table, rowvec vec) {
            colvec c(table.n_rows);
            for(int i=0; i < table.n_rows; ++i) {
                c[i]=sqrt(sum(square(table.row(i)-vec)));
            }
            return c;
        };
        colvec calculateCloseness(colvec id_pos_sep, colvec id_neg_sep) {
            colvec res(id_pos_sep.n_rows, fill::zeros);
            for(int i=0 ; i < id_neg_sep.n_rows; ++i) {
                if(id_neg_sep[0] == 0.0) {
                    res[i]=0;
                } else {
                    res[i]= id_neg_sep[i] / (id_neg_sep[i]+id_pos_sep[i]);
                }
                
            }
            return res;
        };

    public:
        TopsisEngine(int alt_num): alt_num(alt_num),
                                   weights({1.0/4.0, 1.0/4.0, 1.0/4.0, 1.0/4.0}),
                                   attrs({false,true,true,true}),
                                   alt_added(0) {
            table=mat(alt_num, 4, fill::zeros);
        };
        void addAlternative(ps_alt id, int hop, double pdr, double nrg, double env) {
            if(alt_added < alt_num) {
                alts.push_back(id);
                rowvec v({static_cast<double>(hop), pdr, nrg, env});
                table.row(alt_added++)=v;
            } else {
                throw std::runtime_error("No space left");
            }
        };
        void addWeights(std::vector<double> w_arr) {
            weights=w_arr;
        };
        std::vector<ps_alt> getRanking() {
            std::vector<ps_alt> res;
            auto n_table=normalizeMatrix();
            auto n_w_table=applyWeights(n_table);
            auto v_best=calculateIdealBest(n_w_table);
            auto v_worst=calculateIdealWorst(n_w_table);
            auto id_pos_sep=calculateSeparation(n_w_table,v_best);
            auto id_neg_sep=calculateSeparation(n_w_table,v_worst);
            auto closeness=calculateCloseness(id_pos_sep,id_neg_sep);
            uvec index=sort_index(closeness,"descend");
            for(auto it=index.begin() ; it != index.end() ; ++it) {
                alts[*it].rank = closeness[*it];
                res.push_back(alts[*it]);
            }
            return res;
        };
};


#endif // _TOPSIS_H_
