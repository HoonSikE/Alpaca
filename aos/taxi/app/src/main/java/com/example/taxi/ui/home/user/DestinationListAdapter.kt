package com.example.taxi.ui.home.user

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.databinding.ItemDestinationListBinding


class DestinationListAdapter: RecyclerView.Adapter<DestinationListAdapter.DestinationListViewHolder>() {
    private var frequentDestinationList = mutableListOf<FrequentDestination>()
    lateinit var onItemClickListener: (View, String, String, String, String) -> Unit

    fun setListData(data: MutableList<FrequentDestination>){
        frequentDestinationList = data
    }

    fun updateList(list: MutableList<FrequentDestination>){
        this.frequentDestinationList = list
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): DestinationListViewHolder {
        return DestinationListViewHolder(
            ItemDestinationListBinding.inflate(
                LayoutInflater.from(parent.context),
                parent,
                false
            )
        ).apply {
            bindOnItemClickListener(onItemClickListener)
        }
    }

    override fun onBindViewHolder(holder: DestinationListViewHolder, position: Int) {
        holder.bind(frequentDestinationList[position])
    }

    override fun getItemCount(): Int {
        return frequentDestinationList.size
    }

    class DestinationListViewHolder(private val binding: ItemDestinationListBinding) :
        RecyclerView.ViewHolder(binding.root) {

        lateinit var place : String
        lateinit var address : String
        lateinit var latitude : String
        lateinit var longitude : String
        var count : Int = 0

        fun bind(data: FrequentDestination) {
            binding.addrName.text = data.addressName
            place = data.addressName
            address = data.address
            latitude = data.latitude
            longitude = data.longitude
            count = data.count
        }

        fun bindOnItemClickListener(onItemClickListener: (View, String, String, String, String) -> Unit ) {
            binding.root.setOnClickListener {
                onItemClickListener(it, place, address, latitude, longitude)
            }
        }
    }

}