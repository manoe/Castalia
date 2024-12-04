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

struct alt {
    int id;
    double rank;
};

class TopsisEngine {
    private:
        mat table;
        int alt_added;
        std::vector<alt> alts; /* The alternatives */
        rowvec weights; /* Weights */
        std::vector<bool> attrs; /* Benefit or cost attribute? */
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
        TopsisEngine(const int alt_num, const int attr_num): alt_added(0) {
            table=mat(alt_num, attr_num, fill::zeros);
            weights=rowvec(attr_num, fill::value(1.0/static_cast<double>(alt_num)));
            attrs=std::vector<bool>(attr_num, false);
        };

        void addAlternative(const alt id, const std::vector<double> &alt) {
            if(alt.size() != table.n_cols) {
                throw std::invalid_argument("Alternative attribute count does not match table column size");
            }
            if(alt_added < table.n_rows) {
                alts.push_back(id);
                table.row(alt_added++)=rowvec(alt);
            } else {
                throw std::runtime_error("No space left in table");
            }
        };

        void addWeights(const std::vector<double> &w_arr) {
            weights=w_arr;
        };

        void addBenefits(const std::vector<bool> &b_arr) {
            attrs=b_arr;
        };

        std::vector<alt> getRanking() {
            if (alt_added != table.n_rows) {
                throw std::runtime_error("Not enough entries in table");
            }
            std::vector<alt> res;
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
